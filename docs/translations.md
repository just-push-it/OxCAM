# Translations

OxCAM's interface is translated with Qt's tools. Every text the interface shows is in the
sources: the GUI's own texts are wrapped in `tr()` (or marked with `QT_TR_NOOP` /
`QT_TRANSLATE_NOOP` where they sit in tables). Texts that come from the libraries (strategy
names, descriptions, parameters and choices; tool type names) stay plain English there. The GUI
translates them at run time in the contexts `Strategies` and `Tooling`, and
`apps/opencam-gui/i18n/LibraryStrings.cpp` lists them for `lupdate`.

The translations live in `translations/opencam_<language>.ts`, one Qt Linguist file per
language (`opencam_de.ts`, `opencam_pt_BR.ts`). `opencam_en.ts` holds the English plural forms
and is the starting point for new languages.

## Tools

You need Qt's `lupdate` and `lrelease` (Qt 6). Any of these provides them:

- a Qt installation (`<Qt>/<version>/<compiler>/bin`),
- a distribution package (`qt6-tools-dev-tools` on Debian and Ubuntu, `qt6-linguist` on Fedora),
- `pip install PySide6-Essentials` (the executables are in the `PySide6` folder of the
  Python environment's `site-packages`).

CMake finds them on the `PATH`, or in the folder that the cache variable `OCAM_QT_TOOLS_DIR`
names:

```bash
cmake --preset msvc-gui -DOCAM_QT_TOOLS_DIR=C:/path/to/PySide6
```

With `lrelease`, the build compiles every `translations/opencam_*.ts` and embeds it in the
application. Without it, the application builds as before, in English only. Edit > Preferences >
Language lists the languages a build has. A new choice takes effect the next time OxCAM
starts.

## Adding a language

1. Copy `translations/opencam_en.ts` to `translations/opencam_<language>.ts`, for example
   `opencam_fr.ts`. In the copy, set the `language` attribute of `<TS>` to the same code.
2. Configure again so CMake sees the new file, then build the `update_translations` target. It
   brings every `.ts` file up to date with the sources.
3. Translate the file in Qt Linguist (`linguist translations/opencam_fr.ts`) and build.

## After changing texts

- **Interface texts:** build `update_translations`, then translate what it added (marked
  unfinished) in each language.
- **Library texts** (a strategy's label, description, parameters or choices, a tool type): the
  test `gui.library_strings` fails until `LibraryStrings.cpp` is regenerated. Regenerate it, then
  build `update_translations`:

  ```bash
  oxcam-gui --write-library-strings apps/opencam-gui/i18n/LibraryStrings.cpp
  ```

Undo step names ("Undo Rename operation"), toolpath warnings, and error messages from the
libraries are not translated yet.
