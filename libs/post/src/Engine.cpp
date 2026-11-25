// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// The post engine (DL-0039): every load and run gets a QuickJS-ng runtime of its own with a bare
// context (no std/os modules, no Date, no Math.random), a module loader that resolves only
// oxcam: modules, memory and stack limits, and cancellation through the interrupt handler.

#include "ocam/post/Engine.hpp"

#include "NcFields.hpp"
#include "OutputLibrary.hpp"

#include <ocam/foundation/Error.hpp>
#include <ocam/foundation/Units.hpp>

#include <fmt/format.h>
#include <fmt/ranges.h>
#include <quickjs.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <memory>
#include <new>
#include <string_view>
#include <unordered_map>
#include <utility>

namespace ocam::embedded {
extern const std::string_view postManifestSchemaV1;
extern const std::string_view fanucModule;
extern const std::string_view fanucGenericMillJs;
extern const std::string_view fanucGenericMill4AxisJs;
extern const std::string_view fanucGenericLatheJs;
extern const std::string_view haasLatheJs;
extern const std::string_view haasClassicMillJs;
} // namespace ocam::embedded

namespace ocam::post {
namespace {

using detail::kPostErrorTag;
using detail::toText;

constexpr std::string_view kModulePrefix = "oxcam:";

/// Owns one reference to a JSValue.
class Value {
public:
    Value() = default;
    Value(JSContext* ctx, JSValue value) : ctx_(ctx), value_(value) {}
    ~Value() {
        if (ctx_) {
            JS_FreeValue(ctx_, value_);
        }
    }
    Value(Value&& other) noexcept
        : ctx_(std::exchange(other.ctx_, nullptr)), value_(other.value_) {}
    Value& operator=(Value&& other) noexcept {
        if (this != &other) {
            if (ctx_) {
                JS_FreeValue(ctx_, value_);
            }
            ctx_ = std::exchange(other.ctx_, nullptr);
            value_ = other.value_;
        }
        return *this;
    }
    Value(const Value&) = delete;
    Value& operator=(const Value&) = delete;

    JSValue get() const noexcept { return value_; }
    /// Gives up ownership (for functions that take it).
    JSValue release() noexcept {
        ctx_ = nullptr;
        return value_;
    }

private:
    JSContext* ctx_ = nullptr;
    JSValue value_ = JS_UNDEFINED;
};

std::string handlerName(std::string_view op) {
    // NC program ops are snake_case ("tool_change"); handlers are camelCase ("toolChange").
    std::string name;
    bool upper = false;
    for (const char c : op) {
        if (c == '_') {
            upper = true;
        } else {
            name += upper ? static_cast<char>(std::toupper(static_cast<unsigned char>(c))) : c;
            upper = false;
        }
    }
    return name;
}

/// The memory of one runtime, with the post's limit. QuickJS checks its own limit before it
/// calls the allocator and then fails with nothing to report (it cannot allocate the error
/// either); counting here tells a post that ran out of memory from any other failure.
struct Heap {
    std::size_t limit = 0;
    std::size_t used = 0;
    bool exhausted = false;
};

/// Each block starts with its size, so the usable size needs no platform function. 16 bytes
/// keep the blocks as aligned as malloc's.
constexpr std::size_t kBlockHeader = 16;

void* heapAllocate(void* opaque, std::size_t size) {
    auto& heap = *static_cast<Heap*>(opaque);
    if (size > heap.limit - std::min(heap.limit, heap.used) ||
        size > std::numeric_limits<std::size_t>::max() - kBlockHeader) {
        heap.exhausted = true;
        return nullptr;
    }
    auto* block = static_cast<unsigned char*>(std::malloc(kBlockHeader + size));
    if (!block) {
        heap.exhausted = true;
        return nullptr;
    }
    std::memcpy(block, &size, sizeof size);
    heap.used += size;
    return block + kBlockHeader;
}

std::size_t heapBlockSize(const void* ptr) {
    std::size_t size = 0;
    std::memcpy(&size, static_cast<const unsigned char*>(ptr) - kBlockHeader, sizeof size);
    return size;
}

void* heapCalloc(void* opaque, std::size_t count, std::size_t size) {
    if (size != 0 && count > std::numeric_limits<std::size_t>::max() / size) {
        static_cast<Heap*>(opaque)->exhausted = true;
        return nullptr;
    }
    void* ptr = heapAllocate(opaque, count * size);
    if (ptr) {
        std::memset(ptr, 0, count * size);
    }
    return ptr;
}

void heapFree(void* opaque, void* ptr) {
    if (ptr) {
        static_cast<Heap*>(opaque)->used -= heapBlockSize(ptr);
        std::free(static_cast<unsigned char*>(ptr) - kBlockHeader);
    }
}

void* heapRealloc(void* opaque, void* ptr, std::size_t size) {
    if (!ptr) {
        return heapAllocate(opaque, size);
    }
    auto& heap = *static_cast<Heap*>(opaque);
    const std::size_t old = heapBlockSize(ptr);
    if ((size > old && size - old > heap.limit - std::min(heap.limit, heap.used)) ||
        size > std::numeric_limits<std::size_t>::max() - kBlockHeader) {
        heap.exhausted = true;
        return nullptr;
    }
    auto* block = static_cast<unsigned char*>(
        std::realloc(static_cast<unsigned char*>(ptr) - kBlockHeader, kBlockHeader + size));
    if (!block) {
        heap.exhausted = true; // the old block stays valid
        return nullptr;
    }
    std::memcpy(block, &size, sizeof size);
    heap.used = heap.used - old + size;
    return block + kBlockHeader;
}

constexpr JSMallocFunctions kHeapFunctions{&heapCalloc, &heapAllocate, &heapFree, &heapRealloc,
                                           &heapBlockSize};

struct RuntimeDeleter {
    void operator()(JSRuntime* rt) const { JS_FreeRuntime(rt); }
};
struct ContextDeleter {
    void operator()(JSContext* ctx) const { JS_FreeContext(ctx); }
};

class Sandbox;
Sandbox& sandbox(JSContext* ctx) {
    return *static_cast<Sandbox*>(JS_GetContextOpaque(ctx));
}

/// One QuickJS runtime and context for one load or run of a post.
class Sandbox {
public:
    Sandbox(std::string name, const EngineLimits& limits, CancelToken& cancel)
        : name_(std::move(name)), cancel_(cancel) {
        heap_.limit = limits.memory;
        rt_.reset(JS_NewRuntime2(&kHeapFunctions, &heap_));
        if (!rt_) {
            throw Error(ErrorCode::Internal, "the post engine could not start");
        }
        JS_SetMaxStackSize(rt_.get(), limits.stack);
        JS_SetInterruptHandler(rt_.get(), &Sandbox::interrupt, this);
        JS_SetModuleLoaderFunc(rt_.get(), &Sandbox::normalizeModule, &Sandbox::loadModule, this);
        ctx_.reset(JS_NewContextRaw(rt_.get()));
        if (!ctx_) {
            throw Error(ErrorCode::Internal, "the post engine could not start");
        }
        JS_SetContextOpaque(ctx_.get(), this);
        // The language without anything that reads the clock or depends on the garbage
        // collector (Date, performance, WeakRef): the same post and program give the same text.
        if (JS_AddIntrinsicBaseObjects(ctx_.get()) < 0 || JS_AddIntrinsicEval(ctx_.get()) < 0 ||
            JS_AddIntrinsicRegExp(ctx_.get()) < 0 || JS_AddIntrinsicJSON(ctx_.get()) < 0 ||
            JS_AddIntrinsicProxy(ctx_.get()) < 0 || JS_AddIntrinsicMapSet(ctx_.get()) < 0 ||
            JS_AddIntrinsicTypedArrays(ctx_.get()) < 0 || JS_AddIntrinsicPromise(ctx_.get()) < 0) {
            throw Error(ErrorCode::Internal, "the post engine could not start");
        }
        const Value global(ctx(), JS_GetGlobalObject(ctx()));
        const Value math(ctx(), JS_GetPropertyStr(ctx(), global.get(), "Math"));
        JS_SetPropertyStr(ctx(), math.get(), "random",
                          JS_NewCFunction(ctx(), &Sandbox::noRandom, "random", 0));
        if (!detail::registerOutputLibrary(ctx(), output)) {
            throw Error(ErrorCode::Internal, "the post engine could not start");
        }
    }

    ~Sandbox() {
        if (ctx_) {
            for (const auto& [key, atom] : atoms_) {
                JS_FreeAtom(ctx(), atom);
            }
            detail::releaseOutputLibrary(ctx(), output);
        }
    }

    Sandbox(const Sandbox&) = delete;
    Sandbox& operator=(const Sandbox&) = delete;

    JSContext* ctx() const noexcept { return ctx_.get(); }
    const std::string& name() const noexcept { return name_; }

    /// Takes a returned value: throws the pending exception as an Error with `code`.
    Value check(JSValue value, ErrorCode code) {
        if (JS_IsException(value)) {
            fail(code);
        }
        return Value(ctx(), value);
    }

    /// Throws the pending JavaScript exception as an Error with `code` (OperationCancelled when
    /// the run was cancelled).
    [[noreturn]] void fail(ErrorCode code) {
        const Value exception(ctx(), JS_GetException(ctx()));
        if (cancel_.isCancelled()) {
            throw OperationCancelled();
        }
        if (heap_.exhausted) {
            throw Error(code, fmt::format("{}: the post ran out of memory (limit {} MiB)", name_,
                                          heap_.limit >> 20));
        }
        throw Error(code, describe(exception.get()));
    }

    /// Compiles and evaluates the post module; returns its namespace.
    Value evaluate(const std::string& source) {
        JS_UpdateStackTop(rt_.get());
        // JS_Eval reads up to the terminating NUL, which std::string keeps.
        Value compiled = check(JS_Eval(ctx(), source.c_str(), source.size(), name_.c_str(),
                                       JS_EVAL_TYPE_MODULE | JS_EVAL_FLAG_COMPILE_ONLY),
                               ErrorCode::Parse);
        auto* module = static_cast<JSModuleDef*>(JS_VALUE_GET_PTR(compiled.get()));
        if (JS_ResolveModule(ctx(), compiled.get()) < 0) {
            fail(ErrorCode::Parse);
        }
        const Value result = check(JS_EvalFunction(ctx(), compiled.release()), ErrorCode::Parse);
        runJobs(ErrorCode::Parse);
        switch (JS_PromiseState(ctx(), result.get())) {
        case JS_PROMISE_REJECTED:
            JS_Throw(ctx(), JS_PromiseResult(ctx(), result.get()));
            fail(ErrorCode::Parse);
        case JS_PROMISE_PENDING:
            throw Error(ErrorCode::Parse,
                        fmt::format("{}: the module's top-level code did not finish (posts "
                                    "cannot wait for anything)",
                                    name_));
        default:
            break;
        }
        return check(JS_GetModuleNamespace(ctx(), module), ErrorCode::Parse);
    }

    /// The export `key` of a module namespace (undefined when missing).
    Value property(const Value& object, const char* key, ErrorCode code) {
        return check(JS_GetPropertyStr(ctx(), object.get(), key), code);
    }

    /// A JSON copy of a JavaScript value; nullopt for values JSON cannot hold (undefined,
    /// functions).
    std::optional<Json> toJson(const Value& value, ErrorCode code) {
        const Value text =
            check(JS_JSONStringify(ctx(), value.get(), JS_UNDEFINED, JS_UNDEFINED), code);
        if (JS_IsUndefined(text.get())) {
            return std::nullopt;
        }
        const auto s = toText(ctx(), text.get());
        if (!s) {
            fail(code);
        }
        return parseJson(*s, name_);
    }

    Value fromJson(const Json& json, ErrorCode code) { return check(toValue(ctx(), json), code); }

    /// A JavaScript copy of a JSON value (JS_EXCEPTION on failure), built directly: every
    /// instruction passes through here.
    static JSValue toValue(JSContext* ctx, const Json& json) {
        switch (json.type()) {
        case Json::value_t::null:
        case Json::value_t::discarded:
            return JS_NULL;
        case Json::value_t::boolean:
            return JS_NewBool(ctx, json.get<bool>());
        case Json::value_t::number_integer:
            return JS_NewInt64(ctx, json.get<std::int64_t>());
        case Json::value_t::number_unsigned:
            return JS_NewNumber(ctx, static_cast<double>(json.get<std::uint64_t>()));
        case Json::value_t::number_float:
            return JS_NewFloat64(ctx, json.get<double>());
        case Json::value_t::string: {
            const auto& s = json.get_ref<const std::string&>();
            return JS_NewStringLen(ctx, s.data(), s.size());
        }
        case Json::value_t::array: {
            const JSValue array = JS_NewArray(ctx);
            if (JS_IsException(array)) {
                return array;
            }
            std::uint32_t index = 0;
            for (const Json& item : json) {
                const JSValue value = toValue(ctx, item);
                if (JS_IsException(value) ||
                    JS_DefinePropertyValueUint32(ctx, array, index++, value, JS_PROP_C_W_E) < 0) {
                    JS_FreeValue(ctx, array);
                    return JS_EXCEPTION;
                }
            }
            return array;
        }
        case Json::value_t::object:
        case Json::value_t::binary:
            break;
        }
        const JSValue object = JS_NewObject(ctx);
        if (JS_IsException(object)) {
            return object;
        }
        for (const auto& [key, item] : json.items()) {
            const JSValue value = toValue(ctx, item);
            if (JS_IsException(value) ||
                JS_DefinePropertyValueStr(ctx, object, key.c_str(), value, JS_PROP_C_W_E) < 0) {
                JS_FreeValue(ctx, object);
                return JS_EXCEPTION;
            }
        }
        return object;
    }

    /// The atom of a key, made once per run. Keys are string literals or names that live as long
    /// as the run.
    JSAtom atom(const char* key) {
        if (const auto it = atoms_.find(key); it != atoms_.end()) {
            return it->second;
        }
        const JSAtom atom = JS_NewAtom(ctx(), key);
        if (atom != JS_ATOM_NULL) {
            atoms_.emplace(key, atom);
        }
        return atom;
    }

    /// The JavaScript object of an instruction, built from its JSON fields directly
    /// (JS_EXCEPTION on failure).
    JSValue instructionValue(const NcInstr& instruction) {
        class Sink final : public detail::FieldSink {
        public:
            Sink(Sandbox& box, JSValue object) : box_(box) { stack_.push_back(object); }
            void number(const char* key, double v) override {
                define(key, JS_NewFloat64(box_.ctx(), v));
            }
            void integer(const char* key, int v) override {
                define(key, JS_NewInt32(box_.ctx(), v));
            }
            void boolean(const char* key, bool v) override {
                define(key, JS_NewBool(box_.ctx(), v));
            }
            void text(const char* key, std::string_view v) override {
                define(key, JS_NewStringLen(box_.ctx(), v.data(), v.size()));
            }
            void null(const char* key) override { define(key, JS_NULL); }
            void beginObject(const char* key) override { open(key, JS_NewObject(box_.ctx())); }
            void endObject() override { close(); }
            void beginArray(const char* key) override { open(key, JS_NewArray(box_.ctx())); }
            void endArray() override { close(); }
            bool failed = false;

        private:
            /// Adds `value` (an object or array) under `key`, or as the next element of the
            /// array being built when `key` is null, and builds it next.
            void open(const char* key, JSValue value) {
                const JSValue held = JS_IsException(value) ? value : JS_DupValue(box_.ctx(), value);
                if (key) {
                    define(key, held);
                } else {
                    append(held);
                }
                stack_.push_back(value);
                next_.push_back(0);
            }
            void close() {
                JS_FreeValue(box_.ctx(), stack_.back());
                stack_.pop_back();
                next_.pop_back();
            }
            void append(JSValue value) {
                if (failed || JS_IsException(value) || JS_IsException(stack_.back())) {
                    JS_FreeValue(box_.ctx(), value);
                    failed = true;
                    return;
                }
                if (JS_SetPropertyUint32(box_.ctx(), stack_.back(), next_.back()++, value) < 0) {
                    failed = true;
                }
            }
            void define(const char* key, JSValue value) {
                if (failed || JS_IsException(value) || JS_IsException(stack_.back())) {
                    JS_FreeValue(box_.ctx(), value);
                    failed = true;
                    return;
                }
                const JSAtom a = box_.atom(key);
                if (a == JS_ATOM_NULL || JS_DefinePropertyValue(box_.ctx(), stack_.back(), a, value,
                                                                JS_PROP_C_W_E) < 0) {
                    failed = true;
                }
            }
            Sandbox& box_;
            std::vector<JSValue> stack_;         ///< [0] is the caller's
            std::vector<std::uint32_t> next_{0}; ///< The next element's index, per level
        };
        const JSValue object = JS_NewObject(ctx());
        if (JS_IsException(object)) {
            return object;
        }
        Sink sink(*this, object);
        detail::instructionFields(instruction, sink);
        if (sink.failed) {
            JS_FreeValue(ctx(), object);
            return JS_EXCEPTION;
        }
        return object;
    }

    /// The program program.peek() looks into, and the index of the instruction after the one
    /// being handled (0 in programBegin).
    const NcProgram* program = nullptr;
    std::size_t next = 0;

    /// program.peek(k): the k-th instruction after the one being handled (in programBegin: the
    /// k-th of the program), or null past the end and in programEnd.
    static JSValue peek(JSContext* ctx, JSValueConst, int argc, JSValueConst* argv) {
        Sandbox& box = sandbox(ctx);
        double k = 1.0;
        if (argc > 0 && !JS_IsUndefined(argv[0]) &&
            (!JS_IsNumber(argv[0]) || JS_ToFloat64(ctx, &k, argv[0]) < 0)) {
            return JS_ThrowTypeError(ctx, "program.peek: expected a whole number of 1 or more");
        }
        if (!(k >= 1.0) || k != std::floor(k)) {
            return JS_ThrowTypeError(ctx, "program.peek: expected a whole number of 1 or more");
        }
        if (!box.program || k > static_cast<double>(box.program->instrs.size())) {
            return JS_NULL;
        }
        const std::size_t index = box.next + static_cast<std::size_t>(k) - 1;
        if (index >= box.program->instrs.size()) {
            return JS_NULL;
        }
        try {
            return box.instructionValue(box.program->instrs[index]);
        } catch (const std::bad_alloc&) {
            return JS_ThrowOutOfMemory(ctx);
        }
    }

    /// Calls `self.method(arg)`, which must exist.
    void call(const Value& self, const std::string& method, const Value& arg) {
        const JSAtom handler = JS_NewAtom(ctx(), method.c_str());
        if (handler == JS_ATOM_NULL) {
            fail(ErrorCode::InvalidArgument);
        }
        try {
            callAtom(self, handler, method, arg);
        } catch (...) {
            JS_FreeAtom(ctx(), handler);
            throw;
        }
        JS_FreeAtom(ctx(), handler);
    }

    /// Calls `self[handler](arg)`; `method` names it in messages.
    void callAtom(const Value& self, JSAtom handler, std::string_view method, const Value& arg) {
        const Value function =
            check(JS_GetProperty(ctx(), self.get(), handler), ErrorCode::InvalidArgument);
        if (!JS_IsFunction(ctx(), function.get())) {
            throw Error(ErrorCode::InvalidArgument,
                        fmt::format("{}: the post has no handler '{}'", name_, method));
        }
        JSValue args[] = {arg.get()};
        const Value result =
            check(JS_Call(ctx(), function.get(), self.get(), 1, args), ErrorCode::InvalidArgument);
        if (JS_IsPromise(result.get())) {
            throw Error(ErrorCode::InvalidArgument,
                        fmt::format("{}: handler '{}' is async; handlers must finish before "
                                    "they return",
                                    name_, method));
        }
    }

    /// What the post writes (oxcam:post).
    detail::Output output;

private:
    static int interrupt(JSRuntime*, void* opaque) {
        return static_cast<Sandbox*>(opaque)->cancel_.isCancelled() ? 1 : 0;
    }

    static char* normalizeModule(JSContext* ctx, const char*, const char* name, void*) {
        if (std::string_view(name).starts_with(kModulePrefix)) {
            return js_strdup(ctx, name);
        }
        JS_ThrowReferenceError(ctx, "posts can import only oxcam: modules, not '%s'", name);
        return nullptr;
    }

    static JSModuleDef* loadModule(JSContext* ctx, const char* name, void*) {
        if (std::string_view(name) == "oxcam:post") {
            return detail::createPostModule(ctx);
        }
        if (std::string_view(name) == "oxcam:fanuc") {
            return compileModule(ctx, name, embedded::fanucModule);
        }
        JS_ThrowReferenceError(ctx, "there is no module '%s'", name);
        return nullptr;
    }

    /// One of OxCAM's JavaScript modules (the module stays in the context).
    static JSModuleDef* compileModule(JSContext* ctx, const char* name, std::string_view source) {
        try {
            const std::string text(source); // JS_Eval reads up to a terminating NUL
            const JSValue compiled = JS_Eval(ctx, text.c_str(), text.size(), name,
                                             JS_EVAL_TYPE_MODULE | JS_EVAL_FLAG_COMPILE_ONLY);
            if (JS_IsException(compiled)) {
                return nullptr;
            }
            auto* module = static_cast<JSModuleDef*>(JS_VALUE_GET_PTR(compiled));
            JS_FreeValue(ctx, compiled);
            return module;
        } catch (const std::bad_alloc&) {
            JS_ThrowOutOfMemory(ctx);
            return nullptr;
        }
    }

    static JSValue noRandom(JSContext* ctx, JSValueConst, int, JSValueConst*) {
        return JS_ThrowTypeError(
            ctx, "Math.random is not available in posts: the same program must give the same "
                 "output");
    }

    void runJobs(ErrorCode code) {
        JSContext* jobContext = nullptr;
        int status = 0;
        while ((status = JS_ExecutePendingJob(rt_.get(), &jobContext)) > 0) {
        }
        if (status < 0) {
            fail(code);
        }
    }

    /// A string property of an error object, "" when it has none (or reading it fails).
    std::string errorProperty(JSValueConst error, const char* key) {
        const JSValue value = JS_GetPropertyStr(ctx(), error, key);
        if (JS_IsException(value)) {
            JS_FreeValue(ctx(), JS_GetException(ctx()));
            return {};
        }
        const Value owned(ctx(), value);
        if (JS_IsUndefined(value) || JS_IsNull(value)) {
            return {};
        }
        auto text = toText(ctx(), value);
        if (!text) {
            JS_FreeValue(ctx(), JS_GetException(ctx()));
            return {};
        }
        return std::move(*text);
    }

    /// "<file>:<line>:<column>" of the post's first frame in a stack trace, else "<file>".
    std::string location(const std::string& stack) const {
        // QuickJS frames read "    at f (post.js:12:5)" or "    at post.js:12:5".
        const std::string key = name_ + ":";
        for (std::size_t pos = stack.find(key); pos != std::string::npos;
             pos = stack.find(key, pos + 1)) {
            if (pos > 0 && stack[pos - 1] != '(' && stack[pos - 1] != ' ') {
                continue;
            }
            std::size_t end = pos + key.size();
            auto digits = [&] {
                const std::size_t start = end;
                while (end < stack.size() && std::isdigit(static_cast<unsigned char>(stack[end]))) {
                    ++end;
                }
                return stack.substr(start, end - start);
            };
            const std::string line = digits();
            if (line.empty()) {
                continue;
            }
            std::string column;
            if (end < stack.size() && stack[end] == ':') {
                ++end;
                column = digits();
            }
            return column.empty() ? fmt::format("{}:{}", name_, line)
                                  : fmt::format("{}:{}:{}", name_, line, column);
        }
        return name_;
    }

    std::string describe(JSValueConst exception) {
        if (JS_IsError(exception)) {
            const std::string message = errorProperty(exception, "message");
            const Value tag(ctx(), JS_GetPropertyStr(ctx(), exception, kPostErrorTag));
            if (JS_IsException(tag.get())) {
                JS_FreeValue(ctx(), JS_GetException(ctx()));
            } else if (JS_ToBool(ctx(), tag.get()) > 0) {
                return fmt::format("{}: {}", name_, message);
            }
            const std::string type = errorProperty(exception, "name");
            return fmt::format("{}: {}: {}", location(errorProperty(exception, "stack")),
                               type.empty() ? "Error" : type, message);
        }
        auto text = toText(ctx(), exception);
        if (!text) {
            JS_FreeValue(ctx(), JS_GetException(ctx()));
        }
        return fmt::format("{}: uncaught {}", name_, text.value_or("exception"));
    }

    std::string name_;
    CancelToken& cancel_;
    Heap heap_; // declared before rt_: outlives the runtime that allocates from it
    std::unique_ptr<JSRuntime, RuntimeDeleter> rt_;
    std::unique_ptr<JSContext, ContextDeleter> ctx_; // declared after rt_: freed first
    std::unordered_map<const char*, JSAtom> atoms_;
};

} // namespace

detail::Output& detail::outputOf(JSContext* ctx) {
    return sandbox(ctx).output;
}

namespace {
// -- Manifest and options -------------------------------------------------------------------

const SchemaValidator& exportsSchema() {
    static const SchemaValidator schema(
        parseJson(embedded::postManifestSchemaV1, "post-manifest.v1.schema.json"));
    return schema;
}

PostManifest manifestFromJson(const Json& j, const std::string& name) {
    PostManifest m;
    m.id = j.at("id").get<std::string>();
    m.name = j.at("name").get<std::string>();
    m.version = j.at("version").get<std::string>();
    m.engine = j.at("engine").get<int>();
    if (m.engine > kPostEngineVersion) {
        throw Error(ErrorCode::Unsupported,
                    fmt::format("{}: the post needs post engine version {}; this OxCAM has "
                                "version {}",
                                name, m.engine, kPostEngineVersion));
    }
    m.machines = j.at("machines").get<std::vector<std::string>>();
    m.extension = j.value("extension", "nc");
    const Json arcs = j.value("arcs", Json::object());
    m.maxArcSweep = degToRad(arcs.value("maxSweep", 180.0));
    m.arcPlanes = arcs.value("planes", std::vector<std::string>{"xy", "zx", "yz"});
    m.helicalArcs = arcs.value("helical", true);
    m.tolerance = j.value("tolerance", 0.002);
    m.cycles = j.value(
        "cycles",
        std::vector<std::string>{"drill", "dwell", "peck", "chip_break", "tap", "tap_left", "bore"});
    m.tcp = j.value("multiAxis", "tcp") == "tcp";
    if (const auto it = j.find("verify"); it != j.end() && it->is_string()) {
        m.verify = it->get<std::string>();
    }
    m.encoding = j.value("encoding", "ascii");
    m.subSpindle = j.value("subSpindle", false);
    if (const auto it = j.find("workOffsets"); it != j.end()) {
        for (const Json& r : *it) {
            m.workOffsets.push_back({r.at("from").get<int>(), r.at("to").get<int>(),
                                     r.at("code").get<std::string>(), r.value("start", 1),
                                     r.value("name", std::string())});
        }
    } else {
        m.workOffsets = {{1, 6, "G{}", 54, "Standard"}, {7, 54, "G54.1 P{}", 1, "Extended"}};
    }
    // Unnamed ranges: the first is the standard one, the others extended.
    for (std::size_t i = 0; i < m.workOffsets.size(); ++i) {
        std::string& rangeName = m.workOffsets[i].name;
        if (rangeName.empty()) {
            rangeName = i == 0   ? std::string("Standard")
                        : i == 1 ? std::string("Extended")
                                 : fmt::format("Extended {}", i);
        }
    }
    for (std::size_t i = 0; i < m.workOffsets.size(); ++i) {
        const WorkOffsetRange& r = m.workOffsets[i];
        if (r.to < r.from) {
            throw Error(ErrorCode::InvalidArgument,
                        fmt::format("{}: manifest: workOffsets[{}] ends ({}) before it starts ({})",
                                    name, i, r.to, r.from));
        }
        for (std::size_t k = 0; k < i; ++k) {
            if (r.from <= m.workOffsets[k].to && m.workOffsets[k].from <= r.to) {
                throw Error(
                    ErrorCode::InvalidArgument,
                    fmt::format("{}: manifest: workOffsets[{}] and [{}] overlap", name, k, i));
            }
        }
    }
    return m;
}

PostOption::Type parseOptionType(const std::string& type) {
    return type == "number"   ? PostOption::Type::Number
           : type == "choice" ? PostOption::Type::Choice
           : type == "text"   ? PostOption::Type::Text
                              : PostOption::Type::Bool;
}

/// Why `value` cannot be the value of `option`; empty when it can.
std::string optionProblem(const PostOption& option, const Json& value) {
    switch (option.type) {
    case PostOption::Type::Bool:
        return value.is_boolean() ? std::string() : "must be true or false";
    case PostOption::Type::Number: {
        if (!value.is_number()) {
            return "must be a number";
        }
        const double v = value.get<double>();
        if (option.min && v < *option.min) {
            return fmt::format("must be at least {}", *option.min);
        }
        if (option.max && v > *option.max) {
            return fmt::format("must be at most {}", *option.max);
        }
        return {};
    }
    case PostOption::Type::Choice:
        if (value.is_string() &&
            std::ranges::find(option.choices, value.get<std::string>()) != option.choices.end()) {
            return {};
        }
        return fmt::format("must be one of: {}", fmt::join(option.choices, ", "));
    case PostOption::Type::Text:
        return value.is_string() ? std::string() : "must be text";
    }
    return {};
}

std::vector<PostOption> optionsFromJson(const Json& j, const std::string& name) {
    std::vector<PostOption> options;
    for (const Json& o : j) {
        PostOption option;
        option.id = o.at("id").get<std::string>();
        option.label = o.at("label").get<std::string>();
        option.type = parseOptionType(o.at("type").get<std::string>());
        option.scope = o.value("scope", "program") == "operation" ? PostOption::Scope::Operation
                                                                  : PostOption::Scope::Program;
        option.defaultValue = o.at("default");
        option.choices = o.value("choices", std::vector<std::string>{});
        if (o.contains("min")) {
            option.min = o.at("min").get<double>();
        }
        if (o.contains("max")) {
            option.max = o.at("max").get<double>();
        }
        auto invalid = [&](const std::string& why) {
            return Error(ErrorCode::Parse,
                         fmt::format("{}: option '{}': {}", name, option.id, why));
        };
        if (std::ranges::any_of(options, [&](const PostOption& p) { return p.id == option.id; })) {
            throw invalid("defined twice");
        }
        if (option.type == PostOption::Type::Choice && option.choices.empty()) {
            throw invalid("a choice option needs 'choices'");
        }
        if (option.type != PostOption::Type::Choice && !option.choices.empty()) {
            throw invalid("only choice options have 'choices'");
        }
        if (option.type != PostOption::Type::Number && (option.min || option.max)) {
            throw invalid("only number options have 'min' and 'max'");
        }
        if (option.min && option.max && *option.min > *option.max) {
            throw invalid("'min' is larger than 'max'");
        }
        if (const std::string problem = optionProblem(option, option.defaultValue);
            !problem.empty()) {
            throw invalid("the default " + problem);
        }
        options.push_back(std::move(option));
    }
    return options;
}

} // namespace

PostScript PostScript::load(std::string source, std::string name, CancelToken& cancel,
                            const EngineLimits& limits) {
    PostScript script;
    script.source_ = std::move(source);
    script.name_ = std::move(name);
    script.limits_ = limits;

    Sandbox box(script.name_, limits, cancel);
    const Value exports = box.evaluate(script.source_);
    Json doc = Json::object();
    for (const char* key : {"manifest", "options"}) {
        const Value value = box.property(exports, key, ErrorCode::Parse);
        if (JS_IsUndefined(value.get())) {
            throw Error(ErrorCode::Parse,
                        fmt::format("{}: the post does not export '{}'", script.name_, key));
        }
        if (auto json = box.toJson(value, ErrorCode::Parse)) {
            doc[key] = std::move(*json);
        }
    }
    exportsSchema().validate(doc, script.name_);
    const Value constructor = box.property(exports, "default", ErrorCode::Parse);
    if (!JS_IsConstructor(box.ctx(), constructor.get())) {
        throw Error(ErrorCode::Parse,
                    fmt::format("{}: the post's default export must be a class", script.name_));
    }
    script.manifest_ = manifestFromJson(doc.at("manifest"), script.name_);
    script.options_ = optionsFromJson(doc.at("options"), script.name_);
    return script;
}

Json PostScript::optionValues(const Json& values) const {
    return checkedOptions(values, PostOption::Scope::Program);
}

Json PostScript::operationOptionValues(const Json& values) const {
    return checkedOptions(values, PostOption::Scope::Operation);
}

Json PostScript::checkedOptions(const Json& values, PostOption::Scope scope) const {
    if (!values.is_null() && !values.is_object()) {
        throw Error(ErrorCode::InvalidArgument,
                    fmt::format("{}: option values must be an object", name_));
    }
    Json out = Json::object();
    for (const PostOption& option : options_) {
        if (option.scope == scope) {
            out[option.id] = option.defaultValue;
        }
    }
    if (values.is_null()) {
        return out;
    }
    for (const auto& [id, value] : values.items()) {
        const auto it = std::ranges::find(options_, id, &PostOption::id);
        if (it == options_.end()) {
            throw Error(ErrorCode::InvalidArgument,
                        fmt::format("{}: the post has no option '{}'", name_, id));
        }
        if (it->scope != scope) {
            throw Error(ErrorCode::InvalidArgument,
                        fmt::format("{}: option '{}' is set {}", name_, id,
                                    scope == PostOption::Scope::Program ? "per operation"
                                                                        : "for the program"));
        }
        if (const std::string problem = optionProblem(*it, value); !problem.empty()) {
            throw Error(ErrorCode::InvalidArgument,
                        fmt::format("{}: option '{}' {}", name_, id, problem));
        }
        out[id] = value;
    }
    return out;
}

PostResult PostScript::run(const NcProgram& program, const Json& optionValues,
                           CancelToken& cancel) const {
    constexpr ErrorCode kFailed = ErrorCode::InvalidArgument;
    Sandbox box(name_, limits_, cancel);
    // A fresh evaluation, so nothing a load or an earlier run left behind reaches this one.
    const Value exports = box.evaluate(source_);
    const Value constructor = box.property(exports, "default", kFailed);

    // The program as the post sees it: the NC program JSON without its instructions (handed to
    // the handlers one by one, and to program.peek()), with the post, the options and peek().
    Json doc = detail::programHeaderJson(program);
    Json workOffsets = Json::array();
    for (const WorkOffsetRange& r : manifest_.workOffsets) {
        workOffsets.push_back({{"from", r.from},
                               {"to", r.to},
                               {"code", r.code},
                               {"start", r.start},
                               {"name", r.name}});
    }
    doc["post"] = {{"id", manifest_.id},
                   {"name", manifest_.name},
                   {"version", manifest_.version},
                   {"work_offsets", std::move(workOffsets)}};
    doc["options"] = optionValues;
    const Value programInfo = box.fromJson(doc, kFailed);
    if (JS_SetPropertyStr(box.ctx(), programInfo.get(), "peek",
                          JS_NewCFunction(box.ctx(), &Sandbox::peek, "peek", 1)) < 0) {
        box.fail(kFailed);
    }
    box.program = &program;

    const Value context = box.check(JS_NewObject(box.ctx()), kFailed);
    const Value options = box.property(programInfo, "options", kFailed);
    const Value operations = box.property(programInfo, "operations", kFailed);
    const JSValue out = detail::outObject(box.ctx());
    if (JS_SetPropertyStr(box.ctx(), context.get(), "program",
                          JS_DupValue(box.ctx(), programInfo.get())) < 0 ||
        JS_SetPropertyStr(box.ctx(), context.get(), "options",
                          JS_DupValue(box.ctx(), options.get())) < 0 ||
        JS_IsException(out) || JS_SetPropertyStr(box.ctx(), context.get(), "out", out) < 0) {
        box.fail(kFailed);
    }
    JSValue args[] = {context.get()};
    const Value post =
        box.check(JS_CallConstructor(box.ctx(), constructor.get(), 1, args), kFailed);

    // The handler of each op: its name in camelCase, as an atom made once.
    struct Handler {
        std::string name;
        JSAtom atom = JS_ATOM_NULL;
    };
    std::vector<Handler> handlers(static_cast<std::size_t>(NcOp::SpindleSync) + 1); // the last op
    for (std::size_t op = 0; op < handlers.size(); ++op) {
        handlers[op].name = handlerName(toString(static_cast<NcOp>(op)));
    }
    for (Handler& handler : handlers) {
        handler.atom = box.atom(handler.name.c_str()); // the names outlive the run
    }

    box.call(post, "programBegin", programInfo);
    for (std::size_t index = 0; index < program.instrs.size(); ++index) {
        cancel.throwIfCancelled();
        box.next = index + 1;
        const NcInstr& instruction = program.instrs[index];
        // Operation handlers get the operation itself (program.operations[i]).
        const bool operation =
            instruction.op == NcOp::OperationBegin || instruction.op == NcOp::OperationEnd;
        const Value arg = box.check(
            operation ? JS_GetPropertyUint32(box.ctx(), operations.get(),
                                             static_cast<std::uint32_t>(instruction.operation))
                      : box.instructionValue(instruction),
            kFailed);
        const Handler& handler = handlers[static_cast<std::size_t>(instruction.op)];
        box.callAtom(post, handler.atom, handler.name, arg);
    }
    box.program = nullptr;
    box.call(post, "programEnd", programInfo);
    if (manifest_.encoding == "ascii") {
        if (const auto problem = detail::asciiProblem(box.output.text)) {
            throw Error(kFailed, fmt::format("{}: {}", name_, *problem));
        }
    }
    return PostResult{std::move(box.output.text), std::move(box.output.warnings)};
}

std::optional<std::string> workOffsetCode(const PostManifest& manifest, int n) {
    for (const WorkOffsetRange& r : manifest.workOffsets) {
        if (n >= r.from && n <= r.to) {
            std::string code = r.code;
            if (const auto at = code.find("{}"); at != std::string::npos) {
                code.replace(at, 2, std::to_string(r.start + n - r.from));
            }
            return code;
        }
    }
    return std::nullopt;
}

int lastWorkOffset(const PostManifest& manifest) {
    int last = 0;
    for (const WorkOffsetRange& r : manifest.workOffsets) {
        last = std::max(last, r.to);
    }
    return last;
}

std::string describeWorkOffsets(const PostManifest& manifest) {
    std::vector<std::string> parts;
    for (const WorkOffsetRange& r : manifest.workOffsets) {
        const std::string first = *workOffsetCode(manifest, r.from);
        // "G54.1 P1-P48": of a code of several words, the range's end repeats the last one.
        std::string last = *workOffsetCode(manifest, r.to);
        if (const auto space = r.code.substr(0, r.code.find("{}")).rfind(' ');
            space != std::string::npos) {
            last.erase(0, space + 1);
        }
        parts.push_back(r.from == r.to ? fmt::format("{} ({})", first, r.from)
                                       : fmt::format("{}-{} ({}-{})", first, last, r.from, r.to));
    }
    return fmt::format("{}", fmt::join(parts, ", "));
}

LowerOptions lowerOptionsFor(const PostManifest& manifest) {
    LowerOptions options;
    options.cannedCycles = !manifest.cycles.empty();
    for (const std::string& type : manifest.cycles) {
        if (const auto t = toolpath::parseCycleType(type)) {
            options.cycleTypes.push_back(*t);
        }
    }
    options.tcp = manifest.tcp;
    options.arcPlanes.clear();
    for (const std::string& plane : manifest.arcPlanes) {
        options.arcPlanes.push_back(plane == "zx"   ? toolpath::ArcPlane::ZX
                                    : plane == "yz" ? toolpath::ArcPlane::YZ
                                                    : toolpath::ArcPlane::XY);
    }
    options.helicalArcs = manifest.helicalArcs;
    options.maxArcSweep = manifest.maxArcSweep;
    options.tolerance = manifest.tolerance;
    return options;
}

namespace {

const std::vector<PostScript>& builtinPosts() {
    static const std::vector<PostScript> posts = [] {
        struct Source {
            std::string_view file;
            std::string_view text;
        };
        const Source sources[] = {
            {"fanuc-generic-mill.js", embedded::fanucGenericMillJs},
            {"fanuc-generic-mill-4axis.js", embedded::fanucGenericMill4AxisJs},
            {"fanuc-generic-lathe.js", embedded::fanucGenericLatheJs},
            {"haas-lathe.js", embedded::haasLatheJs},
            {"haas-classic-mill.js", embedded::haasClassicMillJs},
        };
        CancelToken cancel;
        std::vector<PostScript> loaded;
        for (const Source& s : sources) {
            loaded.push_back(PostScript::load(std::string(s.text), std::string(s.file), cancel));
        }
        return loaded;
    }();
    return posts;
}

} // namespace

const PostScript& builtinPost(std::string_view id) {
    for (const PostScript& post : builtinPosts()) {
        if (post.manifest().id == id) {
            return post;
        }
    }
    throw Error(ErrorCode::InvalidArgument, fmt::format("there is no built-in post '{}'", id));
}

std::vector<std::string> builtinPostIds() {
    std::vector<std::string> ids;
    for (const PostScript& post : builtinPosts()) {
        ids.push_back(post.manifest().id);
    }
    return ids;
}

} // namespace ocam::post
