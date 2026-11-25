# The output library: `oxcam:post`

```js
import { number, word, code, out, warning, error } from "oxcam:post";
```

Everything a post writes goes through these functions. They are written in C++ for speed and
keep the output rules (number formats, modal suppression, line numbers, comment characters) in
one place. Settings objects are checked strictly: an unknown setting is an error.

## Numbers: `number(settings)`

```js
const xyz = number({ decimals: 3 });            // "10.", "10.5", "-0.25"
const s = number({ decimals: 0, point: "needed" }); // "4775"
xyz.format(12.3456)                             // "12.346"
```

| Setting | Default | Meaning |
|---|---|---|
| `decimals` | required | 0 to 9 digits after the point. |
| `point` | `"always"` | `"always"`: whole numbers end in a point (`10.`); `"needed"`: `10`. |
| `zeros` | `"trim"` | `"trim"` removes trailing zeros (`10.5`); `"keep"` writes them (`10.500`). |
| `scale` | `1` | Multiplied in before rounding (2 for lathe diameters). |

Rounding is half away from zero; `-0` is never written. `format` refuses anything but a finite
number.

## Address words: `word(letter, format, settings)`

A word remembers the last text it wrote and writes nothing when the next is the same (modal
suppression):

```js
const x = word("X", xyz);
x.of(10)     // "X10."
x.of(10.0001) // ""  (the same text)
x.of(null)   // ""  (the NC program's "unchanged"; nothing changes)
x.reset();   // the next of() writes again (after a tool change, a reference return)
x.last       // "10." (null after a reset)
```

`{ modal: false }` writes every time (for words a control needs in each block).

## Code groups: `code()`

The same for modal codes: `g.of("G1")` returns `"G1"`, or `""` when it is the code in effect;
`reset()` and `last` as for words.

## Writing: `out`

| Function | Does |
|---|---|
| `out.block(...words)` | One block: skips empty, `null` and `undefined` words, joins the rest with `out.separator` (default `" "`), adds the line number when numbering is on. A block without words is not written and uses no number. Words must be text: a raw number is an error, so nothing reaches the output unformatted. |
| `out.skippable(...words)` | The same block for block delete: `/N10 M01`. |
| `out.line(text)` | A line as it is, without a number (`%`, headers). |
| `out.comment(text)` | A comment line, never numbered, in `out.commentStyle`. |
| `out.commentText(text)` | The cleaned comment text alone, for comments inside other lines. |
| `out.numbering({ start, step, max })` | Line numbers from `start` (10) in steps of `step` (10), back to `start` after `max` (99999). `out.numbering(null)` turns them off; they are off until a post turns them on. |
| `out.buffer()` | A buffer; `b.text` is what it holds. |
| `out.into(buffer, fn)` | Runs `fn` with everything written going into `buffer` (line numbers continue), then back, also when `fn` throws. Buffers nest. |
| `out.append(buffer)` | Writes a buffer's text where output goes now. |

Settings, read and written as properties:

| Property | Default | Meaning |
|---|---|---|
| `out.separator` | `" "` | Between words (`""` for controls that want `G0X1.Y2.`). |
| `out.commentStyle` | `"parentheses"` | `"parentheses"` `(TEXT)`, `"semicolon"` `; TEXT`, `"none"` (no comments). |
| `out.upperComments` | `false` | Upper-case comment text. |

**Comment cleaning:** `Ø`/`ø` become `D`, `°` becomes `DEG`; `(`, `)`, `;`, line breaks and
other non-ASCII characters are removed.

**Output check:** the program may hold only printable ASCII and tabs, lines ending in `\n`;
anything else stops posting naming the line and the byte.

Buffers serve subprograms (write a pattern once into a buffer, call it, append the buffer
after `M30`) and headers that depend on the body (a tool list, a line count).

## Messages

- `warning(message)`: shown with the program; posting goes on.
- `error(message)`: stops posting; the message is shown as `post.js: message`.
