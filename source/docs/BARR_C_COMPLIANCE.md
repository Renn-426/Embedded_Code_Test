# Barr-C:2018 compliance of the AAI03 robot application

Scope: every file under `app_program/` (the application).  The micro T-Kernel
port, the Pico SDK, lwIP and the CYW43 driver are third-party code and are not
reformatted; the application includes their headers as system headers.

## How the rules are enforced

Three gates run on every build and stop it on any finding:

1. **Automated scan** `tools/barr_check.py` (run by `build_make` before any
   application object is compiled).
2. **Compiler**: kernel-side modules are built as `-std=c99 -pedantic`; all
   application modules with `-Wall -Wextra -Werror -Wshadow -Wsign-compare
   -Wsign-conversion -Wstrict-prototypes -Wmissing-prototypes`.
3. **Unit tests** `make -C tests` for the pure modules.

In addition, `cppcheck --enable=warning,style,performance,portability` reports
no findings on `app_program/`, and the rules marked "review" were checked by
reading every file.

| Rule | Topic | Enforced by |
|---|---|---|
| 1.1.a | C99 | Compiler (`-std=c99 -pedantic` for kernel-side modules; see deviation D1) |
| 1.1.c | Extensions localised | Only the 7 `_hw.c` / `_net.c` driver modules use the SDK's GNU extensions |
| 1.2.a | 80-column lines | Scan |
| 1.3.a/b | Braces always, on their own lines | Scan |
| 1.4.a/b | Parentheses; `&&`/`\|\|` operands parenthesised | Scan (1.4.b), review (1.4.a) |
| 1.5.a/b | Abbreviations, project table | Review, [ABBREVIATIONS.md](ABBREVIATIONS.md) |
| 1.6.a | Every cast commented | Scan (a comment on the cast line or the lines above it) |
| 1.7.a-d | No `auto`, `register`, `goto`, `continue` | Scan |
| 1.8.a-c | `static`, `const`, `volatile` | Compiler (`-Wmissing-prototypes` catches non-static private functions), review |
| 2.1.a-c | Comment formats; no commented-out code | Scan (2.1.b), review |
| 2.2.a-i | Comment content, Doxygen headers | Scan (Doxygen block on every file and function), review |
| 3.1.a-m | Spaces around keywords, operators, commas, semicolons | Scan (3.1.a, b, k, m), review |
| 3.2.a-d | Alignment | Review |
| 3.3.a-c | One statement per line; blank lines; end-of-file marker | Scan (3.3.a, 3.3.c), review |
| 3.4.a-c | 4-space indentation, switch layout | Review |
| 3.5.a | No tabs | Scan |
| 3.6.a/b | LF only, printable characters | Scan |
| 4.1.a-d | Module names lowercase, unique in 8 characters, `main` | Scan |
| 4.2.a-d | One header per source; include guards; no storage in headers | Scan (4.2.a, 4.2.b), review |
| 4.3.a-f | One entity per file; section order; own header first; no unused includes | Scan (4.3.c), compiler, review |
| 4.4.a | Project templates | `templates/module.h`, `templates/module.c` |
| 5.1.a-c | Type names `_t`, typedef only, module prefix | Scan |
| 5.2.a-c | Fixed-width integers; no `short`/`long` | Scan, review |
| 5.3.a-c | No signed bit-fields or bitwise on signed; no signed/unsigned mixing | Compiler (`-Wsign-conversion`, `-Wsign-compare`); no C bit-fields are used at all |
| 5.4.a/b | Avoid floating point | Scan (`float`/`double` are forbidden): all maths is fixed-point (`fix_math`) |
| 5.5.a/b | Struct padding, bit order | No structs cross a bus; every message is serialised field by field with explicit masks and shifts (`bitpack`) |
| 5.6.a/b | `bool`; relational conversion | Scan (`b_` names), review |
| 6.1.a-i | Function names | Scan (6.1.c-f), review |
| 6.2.a-f | Length, single exit, prototypes, `static` | Scan (6.2.a), compiler, review |
| 6.3.a/b | Function-like macros | Review (see deviation D4) |
| 6.4.a | Task functions end in `_task` | Scan |
| 6.5.a-d | ISRs | Review (see deviation D2) |
| 7.1.a-o | Variable names (`g_`, `p_`, `b_`, `h_`, length, case) | Scan, review |
| 7.2.a-d | Initialisation | Compiler (`-Wall` uninitialised-use warnings), cppcheck, review |
| 8.1.a | No comma in declarations | Scan |
| 8.2.a-d | Shortest clause first; nesting at most 2; no assignment in `if`; `else` after `else if` | Scan (8.2.b, 8.2.c), review |
| 8.3.a-c | `switch` layout, `default`, fall-through comments | Scan (8.3.b), review |
| 8.4.a-d | Loop bounds, `for (;;)` | Scan (8.4.c), review |
| 8.5.a/b | No `abort`, `exit`, `setjmp`, `longjmp` | Scan |
| 8.6.a | Constant on the left of `==` | Scan |

## Deviations (each is deliberate and localised)

**D1. Rule 1.1.a, C99, in the driver modules.**  The seven `_hw.c` / `_net.c`
modules include Pico SDK headers, which use C11 `static_assert` and GNU
extensions and do not compile as C99.  They are built as `gnu11`, as rule
1.1.c allows for "a small number of device driver modules that interface
directly to hardware".  The other 23 modules are strict C99.

**D2. Rule 6.5.a, interrupt keyword.**  micro T-Kernel enters every interrupt
through its own assembly entry point and calls a `TA_HLNG` handler as an
ordinary AAPCS function, and the Cortex-M0+ stacks the caller-saved registers
in hardware, so no compiler keyword is needed or correct.  The handlers
(`gpio_irq_bank0_isr`, `period_tick_isr`) are `static`, at the end of their
modules, and named `_isr`.  GPIO pin callbacks that run inside that handler are
also named `_isr`.  The port's vector table already routes unused interrupts to
its default handler (rule 6.5.d).

**D3. Rule 6.1.i, module prefix.**  `clock_get_hz()` and `time_us_64()` in
`sdk_stub_hw.c` keep the names the Pico SDK calls them by.  They are compiled
only in the radio-less bench build.

**D4. Rule 6.3.a, function-like macro.**  `MBF_SLOT_BYTES(SIZE)` in
`robot_ipc.c` sizes static arrays, which requires a constant expression that a
function cannot provide.  It obeys rule 6.3.b (parenthesised, parameter used
once, no control transfer).

**D5. Rule 4.2.a, header without a source.**  `robot_cfg.h` holds only
constants (pins and tuning), so it has no `.c` file.  Every `.c` file has its
`.h` file.

**D6. Rule 5.2.a, kernel types at the API boundary.**  Task entry points and
kernel calls use the kernel's own types (`INT`, `ID`, `ER`, `TMO`, `UINT`)
exactly as the micro T-Kernel API declares them; all application data uses
the fixed-width types.

**D7. Rule 6.2.c, single exit.**  `usermain()` never returns (the kernel shuts
down if it does), and task functions are infinite loops, as rule 6.4's own
example shows.

## Running the scan by hand

```sh
python3 tools/barr_check.py app_program
```

The scan also reports violations in a deliberately bad sample (tabs, missing
braces, `goto`, `long`, `while (1)`, constant on the right of `==`, missing
`default`, unparenthesised `&&` operands, uppercase function names, short
variable names and so on), so it does detect what it claims to.
