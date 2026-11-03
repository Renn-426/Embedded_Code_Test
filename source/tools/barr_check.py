#!/usr/bin/env python3
"""Automated Barr-C (BARR-C:2018) scan for the AAI03 robot application.

The Barr Group Embedded C Coding Standard asks for several rules to be
enforced "via an automated scan during each build".  This script implements
the mechanically checkable subset and is run by build_make before any
application object is compiled.  Rules that need human judgement (comment
quality, cast justification, naming descriptiveness) remain code-review items;
see docs/BARR_C_COMPLIANCE.md.

Usage:  python3 tools/barr_check.py <source-root> [<source-root> ...]
Exit status is 0 when no violation is found and 1 otherwise.
"""

import os
import re
import sys

MAX_LINE = 80
MAX_NAME = 31
EOF_MARKER = '/*** end of file ***/'
# Left over from the kernel demo; not built into the robot.
NOT_ROBOT_FILES = {'demo_tasks.c', 'demo_tasks.h', 'usb_console_compat.h'}

FORBIDDEN_KEYWORDS = {
    'auto': '1.7.a', 'register': '1.7.b', 'goto': '1.7.c',
    'continue': '1.7.d', 'short': '5.2.b', 'long': '5.2.b',
    'float': '5.4.a', 'double': '5.4.a',
}
FORBIDDEN_CALLS = {'abort': '8.5.b', 'exit': '8.5.b', 'setjmp': '8.5.b',
                   'longjmp': '8.5.b'}
SPACED_KEYWORDS = ('if', 'while', 'for', 'switch', 'return')
BLOCK_KEYWORDS = ('if', 'else', 'for', 'while', 'switch', 'do')
C_KEYWORDS = {
    'auto', 'break', 'case', 'char', 'const', 'continue', 'default', 'do',
    'double', 'else', 'enum', 'extern', 'float', 'for', 'goto', 'if',
    'inline', 'int', 'long', 'register', 'restrict', 'return', 'short',
    'signed', 'sizeof', 'static', 'struct', 'switch', 'typedef', 'union',
    'unsigned', 'void', 'volatile', 'while', 'bool', 'true', 'false',
}


class Violation:
    """One rule violation at a file position."""

    def __init__(self, path, line, rule, text):
        self.path = path
        self.line = line
        self.rule = rule
        self.text = text

    def __str__(self):
        return '%s:%d: [Barr-C %s] %s' % (self.path, self.line, self.rule,
                                          self.text)


def strip_code(source):
    """Return (code_lines, comment_lines) with comments/strings blanked.

    Each returned line keeps its length so columns still line up.  String
    and character literals are replaced by quotes around spaces, comments by
    spaces in the code view; the comment view keeps only comment text.
    """
    code = []
    comments = []
    in_block = False
    for raw in source.split('\n'):
        out_code = []
        out_comment = []
        idx = 0
        in_line = False
        quote = ''
        while idx < len(raw):
            char = raw[idx]
            pair = raw[idx:idx + 2]
            if in_block:
                if pair == '*/':
                    in_block = False
                    out_code.append('  ')
                    out_comment.append('  ')
                    idx += 2
                    continue_flag = True
                else:
                    out_code.append(' ')
                    out_comment.append(char)
                    idx += 1
                    continue_flag = True
            elif in_line:
                out_code.append(' ')
                out_comment.append(char)
                idx += 1
                continue_flag = True
            else:
                continue_flag = False
            if continue_flag:
                continue
            if quote:
                if char == '\\':
                    out_code.append('  ')
                    out_comment.append('  ')
                    idx += 2
                elif char == quote:
                    out_code.append(char)
                    out_comment.append(' ')
                    quote = ''
                    idx += 1
                else:
                    out_code.append(' ')
                    out_comment.append(' ')
                    idx += 1
            elif pair == '/*':
                in_block = True
                out_code.append('  ')
                out_comment.append('  ')
                idx += 2
            elif pair == '//':
                in_line = True
                out_code.append('  ')
                out_comment.append('  ')
                idx += 2
            elif char in '"\'':
                quote = char
                out_code.append(char)
                out_comment.append(' ')
                idx += 1
            else:
                out_code.append(char)
                out_comment.append(' ')
                idx += 1
        code.append(''.join(out_code))
        comments.append(''.join(out_comment))
    return code, comments


def check_text_format(path, source, report):
    """Rules 1.2.a, 3.5.a, 3.6.a/b and 3.3.c."""
    if '\r' in source:
        report(1, '3.6.a', 'CR-LF line ending found; use LF only')
    lines = source.split('\n')
    for number, line in enumerate(lines, 1):
        if '\t' in line:
            report(number, '3.5.a', 'tab character in source')
        if len(line) > MAX_LINE:
            report(number, '1.2.a', 'line is %d characters (max %d)'
                   % (len(line), MAX_LINE))
        for char in line:
            if (ord(char) < 32) and (char not in '\t\x0c\r'):
                report(number, '3.6.b', 'non-printing character 0x%02x'
                       % ord(char))
    if not source.endswith('\n\n'):
        report(len(lines), '3.3.c', 'file must end with a blank line')
    content = [line for line in lines if line.strip() != '']
    if (not content) or (content[-1].strip() != EOF_MARKER):
        report(len(lines), '3.3.c', 'last line must be "%s"' % EOF_MARKER)


def check_comments(path, comments, report):
    """Rule 2.1.b: comments never contain /*, // or a backslash."""
    for number, text in enumerate(comments, 1):
        body = text.strip()
        if body == '':
            continue
        inner = body
        if ('//' in inner) or ('/*' in inner) or ('\\' in inner):
            report(number, '2.1.b', 'comment contains /*, // or \\')


def check_braces(path, code, report):
    """Rules 1.3.a and 1.3.b."""
    total = len(code)
    for idx, line in enumerate(code):
        number = idx + 1
        stripped = line.strip()
        if stripped.startswith('#'):
            continue
        if '{' in stripped and stripped != '{':
            initializer = re.search(r'=\s*\{', stripped) is not None
            nested_init = stripped.startswith('{') and \
                (stripped.endswith('},') or stripped.endswith('}'))
            if not (initializer or nested_init):
                report(number, '1.3.b', 'left brace must be on its own line')
        if ('}' in stripped) and (not stripped.startswith('}')):
            if re.search(r'=\s*\{.*\}', stripped) is None and \
                    not stripped.startswith('{'):
                report(number, '1.3.b', 'right brace must start its line')
        if stripped.startswith('}'):
            rest = stripped[1:].strip()
            allowed = rest in ('', ';', ',') or \
                re.fullmatch(r'[a-z0-9_]+_t\s*;', rest) is not None
            if not allowed:
                report(number, '1.3.b', 'right brace must be on its own '
                       'line (found "%s")' % rest)
        match = re.match(r'(?:\}\s*)?(if|for|while|switch|else)\b', stripped)
        if match is None:
            continue
        keyword = match.group(1)
        if keyword == 'else' and re.match(r'else\s+if\b', stripped):
            keyword = 'if'
        end_idx = idx
        if keyword != 'else':
            depth = 0
            started = False
            done = False
            scan = idx
            while (scan < total) and (not done):
                text = code[scan]
                start = text.find(keyword) if scan == idx else 0
                for pos in range(start, len(text)):
                    char = text[pos]
                    if char == '(':
                        depth += 1
                        started = True
                    elif char == ')':
                        depth -= 1
                        if started and (depth == 0):
                            tail = text[pos + 1:].strip()
                            if tail not in ('', ';'):
                                report(scan + 1, '1.3.a', 'statement after '
                                       '"%s (...)" must be in braces'
                                       % keyword)
                            if tail == ';' and keyword != 'while':
                                report(scan + 1, '1.3.a', 'empty statement '
                                       'after "%s"' % keyword)
                            done = True
                            break
                end_idx = scan
                scan += 1
            if (keyword == 'while') and code[end_idx].rstrip().endswith(';'):
                continue
        else:
            if stripped not in ('else', '} else'):
                report(number, '1.3.a', 'statement after "else" must be in '
                       'braces')
        nxt = end_idx + 1
        while (nxt < total) and (code[nxt].strip() == ''):
            nxt += 1
        if (nxt >= total) or (code[nxt].strip() != '{'):
            report(end_idx + 1, '1.3.a', '"%s" block must be followed by a '
                   'brace on the next line' % keyword)


def check_tokens(path, code, report):
    """Rules 1.7, 3.1.a, 3.1.k, 3.1.m, 3.3.a, 5.2.b, 5.4.a, 8.4.c, 8.5.b,
    8.6.a and 8.2.c."""
    for number, line in enumerate(code, 1):
        stripped = line.strip()
        if stripped.startswith('#'):
            continue
        for word in re.findall(r'\b[A-Za-z_][A-Za-z0-9_]*\b', line):
            if word in FORBIDDEN_KEYWORDS:
                report(number, FORBIDDEN_KEYWORDS[word],
                       'forbidden keyword "%s"' % word)
        for name, rule in FORBIDDEN_CALLS.items():
            if re.search(r'\b%s\s*\(' % name, line):
                report(number, rule, 'forbidden library call "%s"' % name)
        for keyword in SPACED_KEYWORDS:
            if re.search(r'\b%s\(' % keyword, line):
                report(number, '3.1.a', '"%s" must be followed by a space'
                       % keyword)
        if re.search(r'\bwhile\s*\(\s*(1|true|TRUE)\s*\)', line):
            report(number, '8.4.c', 'infinite loop must be "for (;;)"')
        if re.search(r'\S\s+;', line) and 'for (' not in line:
            report(number, '3.1.m', 'space before semicolon')
        if re.search(r',(?=\S)', line):
            report(number, '3.1.k', 'comma must be followed by a space')
        no_for = re.sub(r'\bfor\s*\(.*\)', 'for ()', line)
        if (no_for.count(';') > 1) and ('for (' not in no_for):
            report(number, '3.3.a', 'more than one statement on a line')
        if re.search(r'(?:\b[a-z_][a-z0-9_]*|\]|\))\s*(?<![=!<>])==\s*'
                     r'(\d|NULL\b|true\b|false\b|[A-Z][A-Z0-9_]+\b)', line):
            report(number, '8.6.a', 'constant must be on the left of ==')
        cond = re.match(r'\s*(?:\}\s*else\s+)?if\s*\((.*)\)\s*$', line)
        if cond and re.search(r'[^=!<>+\-*/%&|^]=[^=]', cond.group(1)):
            report(number, '8.2.c', 'assignment inside an if test')
        bad_assign = re.search(r'[A-Za-z0-9_\)\]](=|\+=|-=|\*=|/=|%=|&=|'
                               r'\|=|\^=|<<=|>>=)(?!=)', line)
        if bad_assign and (not re.search(r'[=!<>]=', bad_assign.group(0))):
            report(number, '3.1.b', 'assignment operator must be preceded '
                   'by a space')


def is_wrapped(text):
    """True if text is one parenthesised group, e.g. "(a > b)"."""
    if not (text.startswith('(') and text.endswith(')')):
        return False
    depth = 0
    for pos, char in enumerate(text):
        if char == '(':
            depth += 1
        elif char == ')':
            depth -= 1
            if depth == 0 and pos != len(text) - 1:
                return False
    return True


def split_logical(text):
    """Split text at top-level && and || operators."""
    parts = []
    depth = 0
    start = 0
    pos = 0
    while pos < len(text):
        char = text[pos]
        if char in '([':
            depth += 1
        elif char in ')]':
            depth -= 1
        elif depth == 0 and text[pos:pos + 2] in ('&&', '||'):
            parts.append(text[start:pos])
            start = pos + 2
            pos += 1
        pos += 1
    parts.append(text[start:])
    return parts


def check_logical(path, code, report):
    """Rule 1.4.b: operands of && and || are identifiers, constants or
    parenthesised."""
    lines = []
    for line in code:
        lines.append('' if line.strip().startswith('#') else line)
    text = '\n'.join(lines)
    groups = []
    stack = []
    seg_start = 0
    for pos, char in enumerate(text):
        if char == '(':
            stack.append(pos)
        elif char == ')' and stack:
            groups.append((stack.pop() + 1, pos))
        elif char in ';{}' and not stack:
            groups.append((seg_start, pos))
            seg_start = pos + 1
    for start, end in groups:
        body = text[start:end]
        flat = []
        depth = 0
        for char in body:
            if char in '([':
                depth += 1
            elif char in ')]':
                depth -= 1
            flat.append(char if depth == 0 or char in '()[]' else ' ')
        top = ''.join(flat)
        if ('&&' not in top) and ('||' not in top):
            continue
        clauses = []
        depth = 0
        begin = 0
        for pos, char in enumerate(body):
            if char in '([':
                depth += 1
            elif char in ')]':
                depth -= 1
            elif char == ';' and depth == 0:
                clauses.append(body[begin:pos])
                begin = pos + 1
        clauses.append(body[begin:])
        operands = []
        for clause in clauses:
            if ('&&' in clause) or ('||' in clause):
                operands.extend(split_logical(clause))
        for index, operand in enumerate(operands):
            operand = ' '.join(operand.split())
            if index == 0:
                assign = re.match(r'^[^()]*?[^=!<>]=(?!=)\s*(.*)$', operand)
                if assign:
                    operand = assign.group(1)
            if index == len(operands) - 1:
                operand = operand.split('?')[0].strip()
            if operand == '' or is_wrapped(operand) or \
                    re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*|\d\w*', operand):
                continue
            line = text.count('\n', 0, start) + 1
            report(line, '1.4.b', 'operand "%s" of && or || must be in '
                   'parentheses' % operand[:40])


def check_names(path, code, report):
    """Rules 5.1.a/b, 6.1.c/d/e/f, 6.4.a, 6.5.b, 7.1.c/d/e/f/j/k/m."""
    for number, line in enumerate(code, 1):
        stripped = line.strip()
        define = re.match(r'#\s*define\s+([A-Za-z_][A-Za-z0-9_]*)', stripped)
        if define and re.search(r'[a-z]', define.group(1)):
            report(number, '6.1.f', 'macro "%s" contains lowercase letters'
                   % define.group(1))
        if re.match(r'(struct|union|enum)\b[^;]*$', stripped) and \
                not re.match(r'(struct|union|enum)\s+\w+\s*\*', stripped):
            report(number, '5.1.b', 'struct/union/enum must be declared via '
                   'typedef')
        typedef = re.match(r'\}\s*([A-Za-z0-9_]+)\s*;', stripped)
        if typedef and (not typedef.group(1).endswith('_t')):
            report(number, '5.1.a', 'type name "%s" must end in _t'
                   % typedef.group(1))
        simple_typedef = re.match(r'typedef\s+[^;(]*\b([A-Za-z0-9_]+)\s*;',
                                  stripped)
        if simple_typedef and not simple_typedef.group(1).endswith('_t'):
            report(number, '5.1.a', 'type name "%s" must end in _t'
                   % simple_typedef.group(1))
        if re.match(r'[A-Za-z]', line) and (not line.startswith('typedef')):
            function = re.match(r'([a-z_][a-z0-9_]*)\s*\(', line)
            if function and (function.group(1) not in C_KEYWORDS):
                name = function.group(1)
                if name.startswith('_'):
                    report(number, '6.1.c', 'function "%s" begins with an '
                           'underscore' % name)
                if len(name) > MAX_NAME:
                    report(number, '6.1.d', 'function "%s" is longer than %d '
                           'characters' % (name, MAX_NAME))
        definition = re.match(r'(?:static\s+)?(?:[A-Za-z_][A-Za-z0-9_]*\s+\**)*'
                              r'([A-Za-z_][A-Za-z0-9_]*)\s*\([^;]*$', line)
        if definition and (not line.startswith(' ')) and \
                (not definition.group(1).startswith('_Static')) and \
                re.search(r'[a-z]', definition.group(1)) and \
                re.search(r'[A-Z]', definition.group(1)):
            report(number, '6.1.e', 'function "%s" contains uppercase'
                   % definition.group(1))
        file_scope = re.match(r'static\s+(?:volatile\s+|const\s+)*'
                              r'[A-Za-z0-9_]+(?:\s+(?:volatile|const))*'
                              r'\s*(\*?)\s*(?:const\s+)?'
                              r'([A-Za-z_][A-Za-z0-9_]*)\s*(\[|=|;)', line)
        if file_scope and (not line.startswith(' ')):
            pointer, name = file_scope.group(1), file_scope.group(2)
            if not name.startswith('g'):
                report(number, '7.1.j', 'file-scope variable "%s" must '
                       'begin with g' % name)
            if pointer and (not re.match(r'g?pp?_', name)):
                report(number, '7.1.k', 'pointer "%s" must begin with p'
                       % name)
        for decl in re.finditer(r'\b(?:u?int(?:8|16|32|64)_t|bool|ID|ER|'
                                r'char|size_t|int|unsigned)(?=[\s*])\s*(\*?)\s*'
                                r'(?:const\s+)?'
                                r'([a-z_][a-z0-9_]*)\s*(?:=|;|\[|,|\))',
                                line):
            pointer, name = decl.group(1), decl.group(2)
            if name in C_KEYWORDS:
                continue
            if len(name) < 3:
                report(number, '7.1.e', 'variable "%s" is shorter than 3 '
                       'characters' % name)
            if len(name) > MAX_NAME:
                report(number, '7.1.d', 'variable "%s" is longer than %d '
                       'characters' % (name, MAX_NAME))
            if name.startswith('_'):
                report(number, '7.1.c', 'variable "%s" begins with an '
                       'underscore' % name)
            if pointer and (not re.match(r'g?pp?_', name)):
                report(number, '7.1.k', 'pointer "%s" must begin with p'
                       % name)
        for decl in re.finditer(r'\bbool\s+([a-z_][a-z0-9_]*)\s*[=;,)]',
                                line):
            name = decl.group(1)
            if not re.match(r'g?b_', name):
                report(number, '7.1.m', 'Boolean "%s" must begin with b_'
                       % name)
        local = re.match(r'\s+(?:const\s+)?(?:u?int(?:8|16|32|64)_t|bool|ID|'
                         r'ER|[a-z0-9_]+_t)\s+(?:const\s+)?\*?\s*'
                         r'([A-Za-z_][A-Za-z0-9_]*)\s*(?:=|;|\[)', line)
        if local and re.search(r'[A-Z]', local.group(1)):
            report(number, '7.1.f', 'variable "%s" contains uppercase'
                   % local.group(1))


def check_module(path, source, code, report):
    """Rules 2.2.h, 4.2.b, 4.3.c, 6.4.a and 6.5.b."""
    base = os.path.basename(path)
    root, ext = os.path.splitext(base)
    if not source.startswith('/** @file %s' % base):
        report(1, '2.2.h', 'file must start with "/** @file %s"' % base)
    if ext == '.h':
        guard = root.upper() + '_H'
        if ('#ifndef %s' % guard not in source) or \
                ('#define %s' % guard not in source) or \
                ('#endif /* %s */' % guard not in source):
            report(1, '4.2.b', 'header guard %s missing or malformed' % guard)
        for match in re.finditer(r'\}\s*([a-z0-9_]+_t)\s*;|typedef\s+[^;(]*?'
                                 r'\b([a-z0-9_]+_t)\s*;|typedef\s+[^;]*\(\s*\*\s*'
                                 r'([a-z0-9_]+_t)\s*\)', source):
            name = next(group for group in match.groups() if group)
            if not name.startswith(root):
                line = source.count('\n', 0, match.start()) + 1
                report(line, '5.1.c', 'public type "%s" must be prefixed '
                       'with the module name "%s"' % (name, root))
    else:
        if '#include "%s.h"' % root not in source:
            report(1, '4.3.c', 'source must include its own header %s.h'
                   % root)
    lines = source.split('\n')
    for idx, line in enumerate(code):
        match = re.match(r'([a-z_][a-z0-9_]*)\s\(', line)
        if (not match) or line.startswith(' '):
            continue
        name = match.group(1)
        if name in C_KEYWORDS:
            continue
        back = idx - 1
        while (back >= 0) and (lines[back].strip() == ''):
            back -= 1
        while (back >= 0) and re.match(r'(static\s+)?[A-Za-z_]', lines[back]) \
                and ('*/' not in lines[back]):
            back -= 1
        if (back < 0) or (not lines[back].strip().endswith('*/')):
            report(idx + 1, '2.2.h', 'function "%s" lacks a Doxygen comment'
                   % name)
        signature = ' '.join(code[idx:idx + 3])
        if re.search(r'\bINT\s+\w+\s*,\s*void\s*\*', signature) and \
                (not name.endswith('_task')):
            report(idx + 1, '6.4.a', 'task function "%s" must end in _task'
                   % name)
        if name.endswith('_isr') and (not lines[back + 1:idx]):
            pass


def check_structure(path, code, comments, report):
    """Rules 6.2.a (function length), 8.2.b (if nesting), 8.3.b (switch
    default) and 1.6.a (cast comments)."""
    total = len(code)
    idx = 0
    while idx < total:
        line = code[idx]
        if re.match(r'([a-z_][a-z0-9_]*)\s\(', line) and \
                not line.startswith(' '):
            start = idx
            while (idx < total) and (code[idx].strip() != '{'):
                idx += 1
            depth = 0
            body_start = idx
            while idx < total:
                depth += code[idx].count('{') - code[idx].count('}')
                if depth == 0:
                    break
                idx += 1
            length = idx - body_start + 1
            if length > 100:
                report(start + 1, '6.2.a', 'function is %d lines (max 100)'
                       % length)
        idx += 1

    # Nesting of if statements, tracked by brace depth.
    stack = []
    depth = 0
    pending_if = False
    for number, line in enumerate(code, 1):
        stripped = line.strip()
        if re.match(r'(\}\s*)?(else\s+)?if\b', stripped):
            pending_if = not stripped.startswith('else') or \
                stripped.startswith('else if')
            pending_if = True
        elif re.match(r'else\b', stripped):
            pending_if = True
        for char in stripped:
            if char == '{':
                depth += 1
                stack.append(pending_if)
                pending_if = False
                if sum(1 for item in stack if item) > 2:
                    report(number, '8.2.b', 'if statements nested deeper '
                           'than two levels')
            elif char == '}':
                depth -= 1
                if stack:
                    stack.pop()

    # Every switch has a default label.
    text = '\n'.join(code)
    for match in re.finditer(r'\bswitch\s*\(', text):
        pos = text.index('{', match.end())
        level = 0
        end = pos
        while end < len(text):
            if text[end] == '{':
                level += 1
            elif text[end] == '}':
                level -= 1
                if level == 0:
                    break
            end += 1
        if not re.search(r'\bdefault\s*:', text[pos:end]):
            report(text.count('\n', 0, match.start()) + 1, '8.3.b',
                   'switch without a default label')

    # Each cast is explained by a comment on its line or the lines above.
    cast = re.compile(r'\(\s*(?:const\s+)?(?:u?int(?:8|16|32|64)_t|bool|'
                      r'void|INT|UINT|ID|ER|TMO|SZ|PRI|UB|uint16_t|'
                      r'[a-z0-9_]+_t)\s*\*?\s*\)\s*[A-Za-z0-9_(&-]')
    for number, line in enumerate(code, 1):
        if line.strip().startswith('#'):
            continue
        if cast.search(line):
            window = comments[max(0, number - 5):number]
            if not any(text_line.strip() for text_line in window):
                report(number, '1.6.a', 'cast without an explanatory '
                       'comment')


def check_tree(roots):
    """Scan every .c/.h file below the given roots."""
    violations = []
    modules = {}
    for root_dir in roots:
        for dirpath, _dirs, files in os.walk(root_dir):
            for base in sorted(files):
                root, ext = os.path.splitext(base)
                if ext not in ('.c', '.h') or base in NOT_ROBOT_FILES:
                    continue
                path = os.path.join(dirpath, base)

                def report(line, rule, text, path=path):
                    violations.append(Violation(path, line, rule, text))

                if re.fullmatch(r'[a-z0-9_]+', root) is None:
                    report(1, '4.1.a', 'module name must be lowercase '
                           'letters, digits and underscores')
                modules.setdefault(root, {})[ext] = path
                with open(path, 'r', encoding='utf-8', newline='') as handle:
                    source = handle.read()
                code, comments = strip_code(source.replace('\r', ''))
                check_text_format(path, source, report)
                check_comments(path, comments, report)
                check_braces(path, code, report)
                check_tokens(path, code, report)
                check_names(path, code, report)
                check_logical(path, code, report)
                check_structure(path, code, comments, report)
                check_module(path, source, code, report)
    prefixes = {}
    for root, files in modules.items():
        if ('.c' in files) and ('.h' not in files):
            violations.append(Violation(files['.c'], 1, '4.2.a',
                                        'no header %s.h for source' % root))
        key = root[:8]
        if key in prefixes and prefixes[key] != root:
            violations.append(Violation(list(files.values())[0], 1, '4.1.b',
                                        'module "%s" not unique in first 8 '
                                        'characters (clashes with "%s")'
                                        % (root, prefixes[key])))
        prefixes[key] = root
    return violations


def main(argv):
    """Entry point: scan, print every violation, return the exit status."""
    if len(argv) < 2:
        print(__doc__)
        return 2
    violations = check_tree(argv[1:])
    for violation in violations:
        print(violation)
    if violations:
        print('Barr-C scan: %d violation(s)' % len(violations))
        return 1
    print('Barr-C scan: clean')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
