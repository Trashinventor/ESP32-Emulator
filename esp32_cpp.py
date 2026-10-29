"""
Arduino/ESP32 C++ -> Python translator (typed subset) used by ESP32_Simulator.

Supported: preprocessor (#define incl. function-like macros, #if/#ifdef), globals, functions,
if/for/while/do/switch, structs, enums, typedef, arrays (also 2D), static locals, casts,
C integer semantics (truncating division, unsigned wrap-around), String methods, printf-style output.
Unknown library types become permissive dummy objects, unknown functions become logged stubs.

Not supported (reported with a line number): classes with methods, templates, lambdas, new/delete,
goto, references/pointers to scalars (passed by value).
"""
import re
import ast
import keyword


class CppError(Exception):
    def __init__(self, msg, line=0):
        super().__init__(f"line {line}: {msg}" if line else msg)


# --------------------------------------------------------------------------------------------------
# Lexing / preprocessing
# --------------------------------------------------------------------------------------------------
TOKEN_RE = re.compile(r'''
 (?P<ws>\s+)
|(?P<num>0[xX][0-9a-fA-F]+[uUlL]*|0[bB][01]+[uUlL]*|(?:\d+\.\d*|\.\d+|\d+)(?:[eE][+-]?\d+)?[fFuUlL]*)
|(?P<id>[A-Za-z_]\w*)
|(?P<str>"(?:\\.|[^"\\])*")
|(?P<chr>'(?:\\.|[^'\\])+')
|(?P<op><<=|>>=|\.\.\.|->|::|\+\+|--|\+=|-=|\*=|/=|%=|&=|\|=|\^=|<<|>>|<=|>=|==|!=|&&|\|\||[^\s])
''', re.X)


def strip_comments(s):
    out, i, n = [], 0, len(s)
    while i < n:
        c = s[i]
        if c in "\"'":
            j = i + 1
            while j < n and s[j] != c and s[j] != "\n":
                j += 2 if s[j] == "\\" else 1
            out.append(s[i:j + 1])
            i = j + 1
        elif s.startswith("//", i):
            j = s.find("\n", i)
            i = n if j < 0 else j
        elif s.startswith("/*", i):
            j = s.find("*/", i + 2)
            j = n if j < 0 else j + 2
            out.append("\n" * s.count("\n", i, j))
            i = j
        else:
            out.append(c)
            i += 1
    return "".join(out)


def tokenize(text, line0=1):
    toks, line = [], line0
    for m in TOKEN_RE.finditer(text):
        k = m.lastgroup
        v = m.group()
        if k != "ws":
            toks.append([k, v, line])
        line += v.count("\n")
    return toks


def collect_args(toks, i):
    """toks[i] is '(' -> returns (args, index after ')')."""
    depth, args, cur = 0, [], []
    while i < len(toks):
        v = toks[i][1]
        if toks[i][0] == "op" and v in "([{":
            depth += 1
            if depth > 1:
                cur.append(toks[i])
        elif toks[i][0] == "op" and v in ")]}":
            depth -= 1
            if depth == 0:
                if cur or args:
                    args.append(cur)
                return args, i + 1
            cur.append(toks[i])
        elif v == "," and depth == 1 and toks[i][0] == "op":
            args.append(cur)
            cur = []
        else:
            cur.append(toks[i])
        i += 1
    raise CppError("unterminated macro call", toks[-1][2] if toks else 0)


def expand(toks, macros, depth=0):
    out, i = [], 0
    while i < len(toks):
        t = toks[i]
        if t[0] == "id" and t[1] in macros and depth < 24:
            params, body = macros[t[1]]
            if params is None:
                out += expand([[k, v, t[2]] for k, v, _ in body], macros, depth + 1)
                i += 1
                continue
            if i + 1 < len(toks) and toks[i + 1][1] == "(":
                args, j = collect_args(toks, i + 1)
                sub = []
                for k, v, _ in body:
                    if k == "id" and v in params and params.index(v) < len(args):
                        sub += args[params.index(v)]
                    else:
                        sub.append([k, v, t[2]])
                out += expand(sub, macros, depth + 1)
                i = j
                continue
        out.append(t)
        i += 1
    return out


def preprocess(src):
    """Handles directives; returns (tokens, includes)."""
    src = src.replace("\r\n", "\n").replace("\\\n", " ")
    src = strip_comments(src)
    macros, includes, body_lines = {}, [], []
    active = [True]
    taken = [True]

    def cond_value(expr):
        expr = re.sub(r"defined\s*\(?\s*(\w+)\s*\)?", lambda m: "1" if m.group(1) in macros else "0", expr)
        expr = re.sub(r"[A-Za-z_]\w*", lambda m: _macro_num(m.group(0), macros), expr)
        expr = expr.replace("&&", " and ").replace("||", " or ")
        expr = re.sub(r"!(?!=)", " not ", expr)
        expr = re.sub(r"(\d)[uUlL]+", r"\1", expr)
        try:
            return bool(eval(expr, {"__builtins__": {}}, {}))
        except Exception:
            return True

    for ln, line in enumerate(src.split("\n"), 1):
        s = line.strip()
        if not s.startswith("#"):
            body_lines.append(line if all(active) else "")
            continue
        body_lines.append("")
        m = re.match(r"#\s*(\w+)\s*(.*)", s)
        if not m:
            continue
        d, rest = m.group(1), m.group(2)
        if d in ("ifdef", "ifndef", "if"):
            if d == "if":
                v = cond_value(rest)
            else:
                v = (rest.split()[0] in macros) == (d == "ifdef") if rest.split() else False
            active.append(v)
            taken.append(v)
        elif d == "elif":
            v = (not taken[-1]) and cond_value(rest)
            active[-1] = v
            taken[-1] = taken[-1] or v
        elif d == "else":
            active[-1] = not taken[-1]
            taken[-1] = True
        elif d == "endif":
            if len(active) > 1:
                active.pop()
                taken.pop()
        elif not all(active):
            continue
        elif d == "include":
            includes.append(rest.strip().strip('<>"'))
        elif d == "define":
            mm = re.match(r"(\w+)(\(([^)]*)\))?\s*(.*)", rest)
            if mm:
                params = [p.strip() for p in mm.group(3).split(",") if p.strip()] if mm.group(2) else None
                macros[mm.group(1)] = (params, tokenize(mm.group(4), ln))
        elif d == "undef":
            macros.pop(rest.strip(), None)
        elif d == "error":
            raise CppError("#error " + rest, ln)
    toks = tokenize("\n".join(body_lines))
    return expand(toks, macros), includes


def _macro_num(name, macros):
    if name in ("and", "or", "not"):
        return name
    if name in macros and macros[name][0] is None:
        body = " ".join(v for _, v, _ in macros[name][1])
        if re.fullmatch(r"[\d\sxXa-fA-F+\-*/()uUlL]+", body or "x"):
            return "(" + body + ")"
    return "0"


# --------------------------------------------------------------------------------------------------
# Types
# --------------------------------------------------------------------------------------------------
INT = ("int32_t", 0)
UINT = ("uint32_t", 0)
FLT = ("float", 0)
BOOL = ("bool", 0)
STR = ("String", 0)
CHAR = ("char", 0)
VOID = ("void", 0)
UNK = ("?", 0)
UB = {"uint8_t": 0xFF, "uint16_t": 0xFFFF, "uint32_t": 0xFFFFFFFF, "uint64_t": 0xFFFFFFFFFFFFFFFF}
SB = {"int8_t": 8, "int16_t": 16, "int32_t": 32, "int64_t": 64}
INTNAMES = set(UB) | set(SB) | {"char", "bool"}
SIZES = {"int8_t": 1, "uint8_t": 1, "char": 1, "bool": 1, "int16_t": 2, "uint16_t": 2, "int32_t": 4, "uint32_t": 4,
         "float": 4, "int64_t": 8, "uint64_t": 8, "String": 4}
TYPEMAP = {"byte": "uint8_t", "word": "uint16_t", "boolean": "bool", "size_t": "uint32_t", "uint": "uint32_t",
           "double": "float", "string": "String", "ssize_t": "int32_t", "intptr_t": "int32_t",
           "uintptr_t": "uint32_t", "ptrdiff_t": "int32_t", "esp_err_t": "int32_t", "gpio_num_t": "int32_t",
           "adc_unit_t": "int32_t", "adc_channel_t": "int32_t", "adc_atten_t": "int32_t",
           "gpio_mode_t": "int32_t", "TickType_t": "uint32_t", "BaseType_t": "int32_t",
           "UBaseType_t": "uint32_t", "time_t": "int32_t", "useconds_t": "uint32_t", "int": "int32_t",
           "long": "int32_t", "short": "int16_t", "intmax_t": "int64_t", "uintmax_t": "uint64_t"}
BASIC = set(INTNAMES) | set(TYPEMAP) | {"float", "void", "auto", "String", "unsigned", "signed"}
QUAL = {"const", "volatile", "static", "extern", "inline", "constexpr", "register", "IRAM_ATTR", "ICACHE_RAM_ATTR",
        "ARDUINO_ISR_ATTR", "ARDUINO_RUNNING_CORE_ATTR", "WORD_ALIGNED_ATTR", "EXT_RAM_ATTR",
        "FASTRUN", "PROGMEM", "RTC_DATA_ATTR", "DRAM_ATTR", "RTC_NOINIT_ATTR", "mutable", "__inline__"}
MODS = {"unsigned", "signed", "long", "short"}
NOT_TYPE = {"return", "else", "delete", "new", "goto", "case", "throw", "sizeof", "break", "continue", "if", "while",
            "for", "do", "switch", "default", "true", "false", "NULL", "nullptr"}
PY_RESERVED = set(keyword.kwlist) | {"int", "float", "bool", "str", "len", "chr", "ord", "range", "list", "print",
                                     "type", "id", "input", "open", "sum", "all", "any", "math", "time", "dict",
                                     "set", "object", "bytes", "copy", "pow_", "match", "case"}
FLOAT_FUNCS = {"sin", "cos", "tan", "asin", "acos", "atan", "atan2", "sqrt", "pow", "exp", "log", "log10", "fabs",
               "floor", "ceil", "fmod", "sinf", "cosf", "tanf", "sqrtf", "powf", "expf", "logf", "fabsf", "floorf",
               "ceilf", "fmodf", "atan2f", "sinh", "cosh", "tanh", "radians", "degrees", "trunc", "cbrt", "hypot",
               "atof", "esp_timer_get_time_f", "parseFloat", "toFloat", "readVoltage", "temperatureRead"}
STR_FUNCS = {"readString", "readStringUntil", "substring", "c_str", "toUpperCase", "toLowerCase"}
UINT_FUNCS = {"millis", "micros", "esp_random", "xTaskGetTickCount", "getCpuFrequencyMhz", "getFreeHeap"}
INT64_FUNCS = {"esp_timer_get_time"}
ENUM_BASE = ("int32_t", 0)

STR_METHODS = {
    "length": ("len({o})", UINT), "toInt": ("_stoi({o})", INT), "toFloat": ("_stof({o})", FLT),
    "c_str": ("{o}", STR), "charAt": ("_charat({o},{0})", CHAR), "substring": ("{o}[{0}:{1}]", STR),
    "indexOf": ("{o}.find({0}{,1})", INT), "lastIndexOf": ("{o}.rfind({0})", INT),
    "startsWith": ("{o}.startswith({0})", BOOL), "endsWith": ("{o}.endswith({0})", BOOL),
    "equals": ("({o} == {0})", BOOL), "equalsIgnoreCase": ("({o}.lower() == {0}.lower())", BOOL),
    "isEmpty": ("(len({o}) == 0)", BOOL), "compareTo": ("_cmp({o},{0})", INT), "reserve": ("None", VOID),
    "toCharArray": ("None", VOID), "size": ("len({o})", UINT),
}
STR_MUT = {
    "toUpperCase": "{o}.upper()", "toLowerCase": "{o}.lower()", "trim": "{o}.strip()",
    "concat": "({o} + {0})", "replace": "{o}.replace({0},{1})", "remove": "_sremove({o},{0}{,1})",
    "setCharAt": "_ssetchar({o},{0},{1})",
}


def is_str(t):
    return t == STR


def is_float(t):
    return t == FLT


def is_intish(t):
    return t[1] == 0 and (t[0] in INTNAMES or t[0] == "?")


class V:
    __slots__ = ("c", "t", "lv", "inc", "lit")

    def __init__(self, c, t, lv=None, lit=None):
        self.c, self.t, self.lv, self.inc, self.lit = c, t, lv, None, lit


def arith(a, b):
    if a == FLT or b == FLT:
        return FLT
    ba = UB.get(a[0], 0) if a[1] == 0 else 0
    bb = UB.get(b[0], 0) if b[1] == 0 else 0
    if ba >= 0xFFFFFFFF or bb >= 0xFFFFFFFF:
        return ("uint64_t", 0) if max(ba, bb) > 0xFFFFFFFF else UINT
    if a[0] == "int64_t" or b[0] == "int64_t":
        return ("int64_t", 0)
    return INT


class Translator:
    def __init__(self, tokens, known=()):
        self.toks = tokens + [["eof", "", tokens[-1][2] if tokens else 0]] * 8
        self.known = set(known)
        self.funcs, self.gvars, self.structs = {}, {}, {}
        self.enums, self.typedefs = set(), {}
        self.addr_taken = set()
        self.reset_pass()

    def reset_pass(self):
        self.i = 0
        self.out = []
        self.ind = 0
        self.scopes = []
        self.fn = None
        self.assigned = set()
        self.statics = []
        self.loops = []
        self.uid = 0
        self.cells_seen = set()
        self.defined = set(self.funcs)
        self.unknown_calls, self.unknown_ids, self.warnings = set(), set(), []

    addr_taken = None

    def cell_key(self, py, glob):
        return py if glob else (self.fn, py)

    def want_cell(self, py, glob, ty, strict_scalar=True):
        return (self.cell_key(py, glob) in self.addr_taken
                and ty[1] == 0 and (ty[0] in INTNAMES or ty[0] in ("float", "String") or ty[0] in self.enums
                                    or self.unknown_type(ty[0])))

    # ---- token helpers ---------------------------------------------------------------------------
    def peek(self, k=0):
        return self.toks[self.i + k]

    def nxt(self):
        t = self.toks[self.i]
        self.i += 1
        return t

    def is_op(self, v, k=0):
        t = self.toks[self.i + k]
        return t[0] == "op" and t[1] == v

    def is_id(self, v, k=0):
        t = self.toks[self.i + k]
        return t[0] == "id" and t[1] == v

    def accept(self, v):
        t = self.toks[self.i]
        if t[1] == v and t[0] in ("op", "id"):
            self.i += 1
            return True
        return False

    def expect(self, v):
        t = self.toks[self.i]
        if t[1] != v or t[0] not in ("op", "id"):
            raise CppError(f"expected '{v}' but found '{t[1] or 'end of file'}'", t[2])
        self.i += 1

    def err(self, msg):
        raise CppError(msg, self.peek()[2])

    def emit(self, s):
        self.out.append("    " * self.ind + s)

    def new_id(self):
        self.uid += 1
        return self.uid

    def py(self, name):
        return name + "_" if name in PY_RESERVED else name

    # ---- scopes ------------------------------------------------------------------------------------
    def lookup(self, name):
        for d in reversed(self.scopes):
            if name in d:
                return d[name]
        return self.gvars.get(name)

    def declare(self, name, t, dim=None, top=False, cell=False):
        glob = top or not self.scopes
        ent = (t, self.py(name), glob, dim, cell)
        if glob:
            self.gvars[name] = ent
        else:
            self.scopes[-1][name] = ent
        return ent

    def mark_assigned(self, lv):
        if lv and lv[0] == "name" and lv[2]:
            self.assigned.add(lv[1])

    # ---- types ---------------------------------------------------------------------------------------
    def is_known_type(self, n):
        return n in BASIC or n in self.typedefs or n in self.structs or n in self.enums

    def parse_type(self):
        """Returns (type, flags)."""
        fl, uns, sgn, longs, short, name = set(), False, False, 0, False, None
        while True:
            t = self.peek()
            if t[0] != "id":
                break
            v = t[1]
            if v in QUAL:
                fl.add(v)
            elif v == "unsigned":
                uns = True
            elif v == "signed":
                sgn = True
            elif v == "long":
                longs += 1
            elif v == "short":
                short = True
            elif v in ("struct", "enum", "class", "union"):
                pass
            elif name is None:
                if (uns or sgn or longs or short) and not self.is_known_type(v):
                    break
                name = v
                self.i += 1
                while self.is_op("::") and self.peek(1)[0] == "id":
                    self.i += 1
                    name = self.nxt()[1]
                if self.is_op("<") and name not in BASIC:
                    self.skip_angle()
                continue
            else:
                break
            self.i += 1
        if name is None:
            name = "int"
        if name in self.typedefs:
            ty = self.typedefs[name]
        else:
            if name == "char":
                n = "uint8_t" if uns else "char"
            elif name in ("int", None) or name in ("long", "short"):
                n = "int"
            else:
                n = name
            if n == "int":
                n = "int16_t" if short else ("int64_t" if longs >= 2 else "int32_t")
                if uns:
                    n = n.replace("int", "uint", 1) if not n.startswith("u") else n
            else:
                n = TYPEMAP.get(n, n)
                if uns and n in SB:
                    n = "u" + n
            if name == "float" and longs:
                n = "float"
            ty = (n, 0)
        depth = ty[1]
        while True:
            if self.accept("*"):
                depth += 1
                while self.peek()[0] == "id" and self.peek()[1] in ("const", "volatile"):
                    self.i += 1
            elif self.is_op("&") or self.is_op("&&"):
                self.i += 1
                fl.add("ref")
            else:
                break
        return (ty[0], depth), fl

    def skip_angle(self):
        depth = 0
        while True:
            t = self.nxt()
            if t[1] == "<":
                depth += 1
            elif t[1] == ">":
                depth -= 1
            elif t[1] == ">>":
                depth -= 2
            elif t[0] == "eof" or t[1] in (";", "{"):
                self.err("malformed template")
            if depth <= 0:
                return

    def norm(self, ty):
        if ty[0] == "char" and ty[1] >= 1:
            return ("String", ty[1] - 1)
        return ty

    def unknown_type(self, n):
        return not (self.is_known_type(n))

    def default(self, ty):
        n, d = ty
        if d:
            return "None"
        if n == "float":
            return "0.0"
        if n == "bool":
            return "False"
        if n == "String":
            return '""'
        if n in self.structs:
            return f"{n}()"
        if n in INTNAMES or n in self.enums or n == "?":
            return "0"
        return f"_Dummy({n!r})"

    def array_default(self, ty, dims):
        """ty = element type; dims = list of python size expressions."""
        inner = self.default(ty)
        if len(dims) == 1:
            if inner in ("0", "0.0", "False", '""', "None"):
                return f"[{inner}] * {dims[0]}"
            return f"[{inner} for _ in range({dims[0]})]"
        return f"[{self.array_default(ty, dims[1:])} for _ in range({dims[0]})]"

    # ---- coercion ---------------------------------------------------------------------------------
    def conv(self, v, to):
        """python code of v converted to type `to` (assignment / argument semantics)."""
        c, f = v.c, v.t
        if to[1] or to[0] in ("?", "void", "auto") or f[1] and not to[1]:
            return c
        n = to[0]
        if n == "float":
            if f == FLT:
                return c
            return repr(float(v.lit)) if isinstance(v.lit, (int, float)) else f"float({c})"
        if n == "String":
            return self.to_s(v)
        if n == "bool":
            return c if f == BOOL else f"bool({c})"
        if n in INTNAMES or n in self.enums:
            if isinstance(v.lit, (int, float)) and f != STR:
                x = int(v.lit)
                if n in UB:
                    x &= UB[n]
                return str(x)
            if f == FLT:
                c = f"int({c})"
                return f"({c} & {UB[n]})" if n in UB else c
            if f == STR:
                return c
            if n in UB and not (f[0] in UB and UB[f[0]] <= UB[n]):
                return f"({c} & {UB[n]})"
            return c
        if n in self.structs and v.lv is not None:
            return f"_cp({c})"
        return c

    def to_s(self, v):
        t = v.t
        if t == STR:
            return v.c
        if t == CHAR:
            return f"chr({v.c})"
        if t == FLT:
            return f"_sf({v.c},2)"
        if t == BOOL or t[0] == "?":
            return f"_s({v.c})"
        return f"str({v.c})"

    # ---- expressions --------------------------------------------------------------------------------
    def expr(self, stmt=False):
        v = self.assign(stmt)
        while self.is_op(",") and not stmt:
            self.i += 1
            v2 = self.assign(False)
            v = V(f"({v.c}, {v2.c})[1]", v2.t)
        return v

    def assign(self, stmt=False):
        lhs = self.ternary()
        t = self.peek()
        if t[0] == "op" and t[1] in ("=", "+=", "-=", "*=", "/=", "%=", "&=", "|=", "^=", "<<=", ">>="):
            self.i += 1
            if lhs.lv is None:
                raise CppError("assignment to non-lvalue", t[2])
            rhs = self.assign(False)
            if t[1] != "=":
                rhs = self.binop(t[1][:-1], V(lhs.c, lhs.t), rhs)
            code = self.conv(rhs, lhs.t)
            return self.store(lhs, code, stmt)
        if stmt and lhs.inc is not None:
            return lhs.inc
        return lhs

    def store(self, lhs, code, stmt):
        lv = lhs.lv
        self.mark_assigned(lv)
        if lv[0] == "name":
            return V(f"{lv[1]} = {code}" if stmt else f"({lv[1]} := {code})", lhs.t)
        if lv[0] == "sub":
            return V(f"{lv[1]}[{lv[2]}] = {code}" if stmt else f"_seti({lv[1]}, {lv[2]}, {code})", lhs.t)
        return V(f"{lv[1]}.{lv[2]} = {code}" if stmt else f"_seta({lv[1]}, {lv[2]!r}, {code})", lhs.t)

    def ternary(self):
        c = self.binary(0)
        if self.accept("?"):
            a = self.assign(False)
            self.expect(":")
            b = self.assign(False)
            t = arith(a.t, b.t) if is_intish(a.t) and is_intish(b.t) and a.t != b.t else a.t
            if a.t == FLT or b.t == FLT:
                t = FLT
            return V(f"({a.c} if {c.c} else {b.c})", t)
        return c

    PREC = [("||",), ("&&",), ("|",), ("^",), ("&",), ("==", "!="), ("<", ">", "<=", ">="), ("<<", ">>"),
            ("+", "-"), ("*", "/", "%")]

    def binary(self, lvl):
        if lvl >= len(self.PREC):
            return self.unary()
        left = self.binary(lvl + 1)
        while self.peek()[0] == "op" and self.peek()[1] in self.PREC[lvl]:
            op = self.nxt()[1]
            right = self.binary(lvl + 1)
            left = self.binop(op, left, right)
        return left

    def binop(self, op, a, b):
        if op == "&&":
            return V(f"({a.c} and {b.c})", BOOL)
        if op == "||":
            return V(f"({a.c} or {b.c})", BOOL)
        if op in ("==", "!=", "<", ">", "<=", ">="):
            ac, bc = a.c, b.c
            if a.t == STR and b.t == CHAR:
                bc = f"chr({bc})"
            return V(f"({ac} {op} {bc})", BOOL)
        if op == "+" and (a.t == STR or b.t == STR):
            return V(f"({self.to_s(a)} + {self.to_s(b)})", STR)
        t = arith(a.t, b.t)
        ac, bc = a.c, b.c
        if isinstance(a.lit, (int, float)) and isinstance(b.lit, (int, float)) and op in "+-*":
            r = eval(f"{a.lit}{op}{b.lit}")
            if t[0] in UB:
                r &= UB[t[0]]
            return V(repr(r), t, lit=r)
        if op == "/":
            if t == FLT:
                return V(f"({ac} / {bc})", FLT)
            return V(f"({ac} // {bc})" if t[0] in UB else f"_idiv({ac}, {bc})", t)
        if op == "%":
            return V(f"({ac} % {bc})" if t[0] in UB else f"_imod({ac}, {bc})", t)
        if op in ("<<", ">>"):
            t = a.t if is_intish(a.t) and a.t[0] != "bool" else INT
            if a.t[0] in ("uint8_t", "uint16_t", "int8_t", "int16_t", "char", "bool"):
                t = INT
        code = f"({ac} {op} {bc})"
        if t[0] in UB and op in ("+", "-", "*", "<<"):
            code = f"({code} & {UB[t[0]]})"
        return V(code, t)

    def is_cast_ahead(self):
        if not self.is_op("("):
            return False
        t = self.peek(1)
        if t[0] != "id":
            return False
        if t[1] in QUAL or t[1] in MODS or self.is_known_type(t[1]):
            j = 1
            while self.peek(j)[0] == "id" or self.peek(j)[1] in ("*", "::"):
                j += 1
            return self.peek(j)[1] == ")" and self.peek(j + 1)[1] != ""
        return False

    def unary(self):
        t = self.peek()
        if t[0] == "op":
            v = t[1]
            if v in ("++", "--"):
                self.i += 1
                x = self.unary()
                return self.incdec(x, 1 if v == "++" else -1, True)
            if v == "-":
                self.i += 1
                x = self.unary()
                if isinstance(x.lit, (int, float)):
                    return V(repr(-x.lit), x.t, lit=-x.lit)
                tt = x.t if x.t == FLT else arith(x.t, INT)
                return V(f"(-{x.c})" if tt[0] not in UB else f"((-{x.c}) & {UB[tt[0]]})", tt)
            if v == "+":
                self.i += 1
                return self.unary()
            if v == "!":
                self.i += 1
                return V(f"(not {self.unary().c})", BOOL)
            if v == "~":
                self.i += 1
                x = self.unary()
                tt = arith(x.t, INT)
                return V(f"(~{x.c})" if tt[0] not in UB else f"((~{x.c}) & {UB[tt[0]]})", tt)
            if v == "*":
                self.i += 1
                x = self.unary()
                et = (x.t[0], x.t[1] - 1) if x.t[1] else UNK
                return V(f"{x.c}[0]", et, lv=("sub", x.c, "0"))
            if v == "&":
                self.i += 1
                x = self.unary()
                lv = x.lv
                if lv and lv[0] == "attr" and lv[2] == "v" and lv[1] in self.cells_seen:
                    return V(lv[1], (x.t[0], x.t[1] + 1))  # address of a boxed variable = the box itself
                if lv and lv[0] == "name":
                    self.addr_taken.add(self.cell_key(lv[1], lv[2]))
                return V(x.c, x.t if x.t[1] or x.t[0] not in INTNAMES | {"float"} else (x.t[0], 1))
            if v == "(" and self.is_cast_ahead():
                self.i += 1
                ty, _ = self.parse_type()
                self.expect(")")
                x = self.unary()
                return self.cast(x, self.norm(ty))
        if t[0] == "id" and t[1] == "sizeof":
            return self.sizeof()
        return self.postfix(self.primary())

    def cast(self, x, ty):
        n, d = ty
        if d or n in ("?", "void"):
            return V(x.c, ty)
        if n == "float":
            return V(x.c if x.t == FLT else f"float({x.c})", FLT)
        if n == "bool":
            return V(f"bool({x.c})", BOOL)
        if n == "String":
            return V(self.to_s(x), STR)
        if n in INTNAMES or n in self.enums:
            c = f"int({x.c})" if x.t == FLT else x.c
            if n in UB:
                if isinstance(x.lit, int):
                    return V(str(x.lit & UB[n]), ty, lit=x.lit & UB[n])
                if not (x.t[0] in UB and x.t[1] == 0 and UB[x.t[0]] <= UB[n]):
                    c = f"({c} & {UB[n]})"
            elif n in ("int8_t", "int16_t", "int32_t") and (x.t[0] in UB or x.t[0] in SB and SB[x.t[0]] > SB[n]):
                if not (x.t[0] in ("uint8_t", "uint16_t") and n == "int32_t"):
                    c = f"_i{SB[n]}({c})"
            return V(c, ty)
        return V(x.c, ty)

    def sizeof(self):
        self.i += 1
        paren = self.accept("(")
        if self.is_known_type(self.peek()[1]) or self.peek()[1] in QUAL | MODS:
            ty, _ = self.parse_type()
            ty = self.norm(ty)
            sz = 4 if ty[1] else SIZES.get(ty[0], 4)
            if ty[0] == "float" and ty[1] == 0:
                sz = 4
            r = V(str(sz), UINT, lit=sz)
        else:
            x = self.unary() if not paren else self.assign()
            ent = self.lookup(self.toks[self.i - 1][1]) if x.lv and x.lv[0] == "name" else None
            if x.t[1] or x.t == STR and ent and ent[3]:
                et = (x.t[0], x.t[1] - 1)
                esz = 4 if et[1] else SIZES.get(et[0], 4)
                if x.t == STR and ent and ent[3]:
                    r = V(f"{ent[3]}", UINT)
                else:
                    r = V(f"(len({x.c}) * {esz})", UINT)
            else:
                r = V(str(SIZES.get(x.t[0], 4)), UINT)
        if paren:
            self.expect(")")
        return r

    def incdec(self, x, d, prefix):
        if x.lv is None:
            self.err("++/-- needs an lvalue")
        lv = x.lv
        self.mark_assigned(lv)
        mask = UB.get(x.t[0]) if x.t[1] == 0 else None
        if lv[0] == "name":
            new = f"(({lv[1]} + {d}) & {mask})" if mask else f"{lv[1]} + {d}"
            stmt = f"{lv[1]} = {new}" if mask else f"{lv[1]} {'+' if d > 0 else '-'}= 1"
            expr = f"({lv[1]} := {new})"
            val = expr if prefix else f"({expr} - {d})" if not mask else f"_post({lv[1]}, {expr})"
        elif lv[0] == "sub":
            stmt = f"{lv[1]}[{lv[2]}] += {d}"
            val = f"_pinc({lv[1]}, {lv[2]}, {d}, {1 if prefix else 0})"
        else:
            stmt = f"{lv[1]}.{lv[2]} += {d}"
            val = f"_pinca({lv[1]}, {lv[2]!r}, {d}, {1 if prefix else 0})"
        r = V(val, x.t)
        r.inc = V(stmt, x.t)
        return r

    def primary(self):
        t = self.nxt()
        k, v = t[0], t[1]
        if k == "num":
            return self.number(v, t[2])
        if k == "str":
            s = v
            while self.peek()[0] == "str":
                s = s[:-1] + self.nxt()[1][1:]
            return V(s, STR)
        if k == "chr":
            try:
                ch = ast.literal_eval(v)
            except Exception:
                raise CppError(f"bad char literal {v}", t[2])
            return V(str(ord(ch[0])), CHAR, lit=ord(ch[0]))
        if k == "op":
            if v == "(":
                e = self.expr()
                self.expect(")")
                return V(f"({e.c})" if not e.c.startswith("(") else e.c, e.t, e.lv, e.lit)
            if v == "{":
                self.i -= 1
                return V(self.init_value(("?", 1)), ("?", 1))
            raise CppError(f"unexpected '{v}'", t[2])
        if k == "id":
            if v == "true":
                return V("True", BOOL, lit=1)
            if v == "false":
                return V("False", BOOL, lit=0)
            if v in ("NULL", "nullptr"):
                return V("None", ("void", 1))
            if v in ("static_cast", "reinterpret_cast", "const_cast", "dynamic_cast"):
                self.expect("<")
                ty, _ = self.parse_type()
                self.expect(">")
                self.expect("(")
                x = self.assign()
                self.expect(")")
                return self.cast(x, self.norm(ty))
            if v in ("new", "delete", "goto", "template", "namespace", "try", "catch", "throw", "class"):
                raise CppError(f"'{v}' is not supported by the C++ translator", t[2])
            while self.is_op("::") and self.peek(1)[0] == "id":
                self.i += 1
                v = self.nxt()[1]
            if self.is_op("("):
                return self.call(v, t[2])
            if (self.is_known_type(v) and v not in self.gvars and v != "String") or v == "String" and False:
                pass
            ent = self.lookup(v)
            if ent:
                if ent[4]:
                    self.cells_seen.add(ent[1])
                    return V(f"{ent[1]}.v", ent[0], lv=("attr", ent[1], "v"))
                return V(ent[1], ent[0], lv=("name", ent[1], ent[2]))
            if v in self.funcs:
                return V(self.py(v), UNK)
            if v not in self.known:
                self.unknown_ids.add(v)
            return V(self.py(v), UNK if v[:1].isupper() and not v.isupper() else INT)
        raise CppError(f"unexpected token '{v}'", t[2])

    def number(self, s, line):
        low = s.lower()
        isf = False
        if low.startswith(("0x", "0b")):
            body = re.sub(r"[ul]+$", "", low)
            val = int(body, 0)
        elif re.search(r"[.e]", low) or low.endswith("f") and not low.startswith("0x"):
            val = float(re.sub(r"[fl]+$", "", low))
            isf = True
        else:
            body = re.sub(r"[ul]+$", "", low)
            val = int(body, 8) if len(body) > 1 and body.startswith("0") else int(body)
        if isf:
            return V(repr(val), FLT, lit=val)
        uns = "u" in low[len(re.sub(r"[ul]+$", "", low)):]
        if val > 0xFFFFFFFF:
            t = ("uint64_t", 0) if val > 0x7FFFFFFFFFFFFFFF else ("int64_t", 0)
        elif uns or val > 0x7FFFFFFF:
            t = UINT
        else:
            t = INT
        return V(str(val), t, lit=val)

    def postfix(self, x):
        while True:
            t = self.peek()
            if t[0] != "op":
                return x
            v = t[1]
            if v == "[":
                self.i += 1
                idx = self.expr()
                self.expect("]")
                if x.t == STR:
                    x = V(f"_charat({x.c}, {idx.c})", CHAR)
                else:
                    et = (x.t[0], x.t[1] - 1) if x.t[1] else UNK
                    x = V(f"{x.c}[{idx.c}]", et, lv=("sub", x.c, idx.c))
            elif v in (".", "->"):
                self.i += 1
                name = self.nxt()[1]
                if self.is_op("("):
                    x = self.method(x, name)
                else:
                    ft = UNK
                    if x.t[0] in self.structs:
                        for fn, fty in self.structs[x.t[0]]:
                            if fn == name:
                                ft = fty
                    x = V(f"{x.c}.{self.py(name)}", ft, lv=("attr", x.c, self.py(name)))
            elif v in ("++", "--"):
                self.i += 1
                x = self.incdec(x, 1 if v == "++" else -1, False)
            else:
                return x

    def args(self):
        self.expect("(")
        res = []
        if not self.accept(")"):
            while True:
                res.append(self.assign(False))
                if not self.accept(","):
                    break
            self.expect(")")
        return res

    def method(self, obj, name):
        a = self.args()
        if obj.t == STR:
            if name in STR_MUT and obj.lv:
                tpl = STR_MUT[name]
                code = self.fill(tpl, obj.c, a, True)
                return self.store(obj, code, False)
            if name in STR_METHODS:
                tpl, rt = STR_METHODS[name]
                return V(self.fill(tpl, obj.c, a, True), rt)
        codes = [(f"chr({x.c})" if x.t == CHAR and name in ("print", "println") else x.c) for x in a]
        rt = FLT if name in FLOAT_FUNCS else STR if name in STR_FUNCS else INT
        mname = name + "_" if keyword.iskeyword(name) else name  # library methods keep names like print/len
        return V(f"{obj.c}.{mname}({', '.join(codes)})", rt)

    def fill(self, tpl, o, a, strargs=False):
        cs = [(f"chr({x.c})" if x.t == CHAR else x.c) for x in a]

        def rep(m):
            if m.group(1) == "o":
                return o
            if m.group(2):  # optional {,N}
                n = int(m.group(2))
                return f", {cs[n]}" if n < len(cs) else ""
            n = int(m.group(1))
            return cs[n] if n < len(cs) else "None" if "[" in tpl else ""
        return re.sub(r"\{(o|\d+)?(?:,(\d+))?\}", lambda m: rep(m) if (m.group(1) or m.group(2)) else m.group(0), tpl)

    def call(self, name, line):
        a = self.args()
        if name in self.typedefs or (name in BASIC and name != "String"):
            ty = self.norm(self.typedefs.get(name, (TYPEMAP.get(name, name), 0)))
            return self.cast(a[0] if a else V("0", INT, lit=0), ty)
        if name == "String":
            if not a:
                return V('""', STR)
            if len(a) == 2 and a[0].t == FLT:
                return V(f"_sf({a[0].c},{a[1].c})", STR)
            if len(a) == 2:
                return V(f"_sbase({a[0].c},{a[1].c})", STR)
            return V(self.to_s(a[0]), STR)
        if name == "dtostrf" and len(a) >= 4 and a[3].lv:
            return self.store(a[3], f"_dtostrf({a[0].c}, {a[1].c}, {a[2].c})", False)
        if name in ("snprintf", "sprintf") and a and a[0].lv:
            rest = a[2:] if name == "snprintf" else a[1:]
            fmt = ", ".join([x.c for x in rest])
            code = f"_cfmt({fmt})"
            return V(f"len({self.store(a[0], code, False).c})", INT)
        if name in ("strcpy", "strncpy") and a and a[0].lv:
            return self.store(a[0], a[1].c, False)
        if name == "strcat" and a and a[0].lv:
            return self.store(a[0], f"({a[0].c} + {a[1].c})", False)
        if name in ("strlen",):
            return V(f"len({a[0].c})", UINT)
        if name in ("abs", "min", "max", "constrain", "sq"):
            t = a[0].t
            for x in a[1:]:
                t = arith(t, x.t)
            return V(f"{name}({', '.join(x.c for x in a)})", t)
        f = self.funcs.get(name)
        if f:
            ret, params = f
            codes = []
            for k, x in enumerate(a):
                codes.append(self.conv(x, params[k]) if k < len(params) else x.c)
            return V(f"{self.py(name)}({', '.join(codes)})", ret)
        ent = self.lookup(name)
        if ent:  # function pointer variable
            return V(f"{ent[1]}({', '.join(x.c for x in a)})", UNK)
        if name not in self.known:
            self.unknown_calls.add(name)
        rt = FLT if name in FLOAT_FUNCS else UINT if name in UINT_FUNCS else ("int64_t", 0) if name in INT64_FUNCS else INT
        codes = [(f"chr({x.c})" if x.t == CHAR and name in ("print", "println") else x.c) for x in a]
        return V(f"{self.py(name)}({', '.join(codes)})", rt)

    # ---- initialisers / declarations ---------------------------------------------------------------
    def init_value(self, ty):
        if self.is_op("{"):
            self.i += 1
            items = []
            sname = ty[0] if ty[1] == 0 and ty[0] in self.structs else None
            elem = (ty[0], ty[1] - 1) if ty[1] > 0 else UNK
            while not self.is_op("}"):
                if sname and len(items) < len(self.structs[sname]):
                    items.append(self.init_value(self.structs[sname][len(items)][1]))
                else:
                    items.append(self.init_value(elem))
                if not self.accept(","):
                    break
            self.expect("}")
            if sname:
                return f"{sname}({', '.join(items)})"
            return "[" + ", ".join(items) + "]"
        v = self.assign(False)
        return self.conv(v, ty)

    def is_decl(self):
        j = self.i
        toks = self.toks
        seen = False
        has_mod = False
        while toks[j][0] == "id" and (toks[j][1] in QUAL or toks[j][1] in MODS or toks[j][1] in ("struct", "enum", "class", "union")):
            has_mod = has_mod or toks[j][1] in MODS
            j += 1
            seen = True
        t = toks[j]
        if t[0] != "id":
            return False
        n = t[1]
        if n in NOT_TYPE:
            return False
        if has_mod and not self.is_known_type(n) and toks[j + 1][1] != "(":
            return True  # e.g. "unsigned long now = ..." / "long x;"
        if self.is_known_type(n):
            return toks[j + 1][1] != "(" or n in ("unsigned", "signed") or seen or toks[j + 2][1] == "*" or \
                toks[j + 2][0] == "id"
        if seen and toks[j + 1][0] == "id":
            return True
        j += 1
        while toks[j][1] == "::" and toks[j + 1][0] == "id":
            j += 2
        if toks[j][1] == "<":
            depth, k = 0, j
            while k < j + 40 and toks[k][1] not in (";", "{", "(") and toks[k][0] != "eof":
                depth += 1 if toks[k][1] == "<" else -1 if toks[k][1] == ">" else -2 if toks[k][1] == ">>" else 0
                k += 1
                if depth <= 0:
                    break
            j = k
        while toks[j][1] in ("*", "&", "&&"):
            j += 1
        if toks[j][0] == "id" and toks[j][1] not in NOT_TYPE and toks[j][1] not in QUAL:
            return toks[j + 1][1] in ("=", ";", "[", "(", ",", "{")
        return False

    def declaration(self, top=False):
        base, fl = self.parse_type()
        static = "static" in fl and not top
        first = True
        while True:
            ty = base
            while self.accept("*"):
                ty = (ty[0], ty[1] + 1)
            if self.is_op("(") and self.is_op("*", 1):  # function pointer
                self.i += 2
                name = self.nxt()[1]
                self.expect(")")
                self.skip_balanced()
                self.finish_decl(name, UNK, [], None, static, top)
            else:
                if self.peek()[0] != "id":
                    self.err("expected a variable name")
                name = self.nxt()[1]
                if top and first and self.is_op("("):
                    return self.function(base, name)
                dims = []
                while self.accept("["):
                    dims.append(None if self.is_op("]") else self.expr().c)
                    self.expect("]")
                ty = self.norm((ty[0], ty[1] + len(dims)))
                if ty == STR or (ty[0] == "String" and ty[1] == len(dims) - 0 and False):
                    pass
                # char buf[N] -> String (dim kept for sizeof)
                strdim = dims[0] if base[0] == "char" and base[1] == 0 and len(dims) >= 1 and ty == STR else None
                self.finish_decl(name, ty, dims if not strdim else [], strdim, static, top, base)
            first = False
            if not self.accept(","):
                break
        self.expect(";")

    def skip_balanced(self):
        if not (self.is_op("(") or self.is_op("{")):
            return
        depth = 0
        while True:
            t = self.nxt()
            if t[1] in ("(", "{") and t[0] == "op":
                depth += 1
            elif t[1] in (")", "}") and t[0] == "op":
                depth -= 1
                if depth == 0:
                    return
            elif t[0] == "eof":
                self.err("unbalanced parentheses")

    def finish_decl(self, name, ty, dims, strdim, static, top, base=None):
        ent_type = ty
        init = None
        n = ty[0]
        unknown = ty[1] == 0 and not dims and self.unknown_type(n) and n not in ("?",)
        if unknown and (self.is_op("(") or self.is_op("{") and False):
            a = self.args() if n in self.known else (self.skip_balanced() or [])
            init = f"{n}({', '.join(x.c for x in a)})" if n in self.known else f"_Dummy({n!r})"
        elif self.accept("="):
            if unknown and (self.peek()[1] == n or self.is_op("{")):
                while not (self.peek()[1] in (",", ";") or self.peek()[0] == "eof"):
                    if self.is_op("(") or self.is_op("{"):
                        self.skip_balanced()
                    else:
                        self.i += 1
                init = f"_Dummy({n!r})"
            elif dims or (ty[1] and self.is_op("{")):
                init = self.init_value((n, ty[1]) if dims is None else (ty[0], ty[1]))
            else:
                init = self.init_value(ty)
        elif ty[1] == 0 and n in self.structs and self.is_op("("):
            a = self.args()
            init = f"{n}({', '.join(x.c for x in a)})"
        elif self.is_op("(") and ty[1] == 0 and n in INTNAMES | {"float"}:
            a = self.args()
            init = self.conv(a[0], ty) if a else None
        if dims:
            elem = (ty[0], ty[1] - len(dims))
            sizes = [d if d is not None else None for d in dims]
            if init is not None and sizes[0] is not None and len(sizes) == 1:
                init = f"_pad({init}, {sizes[0]}, {self.default(elem)})"
            elif init is None:
                if sizes[0] is None:
                    sizes[0] = "0"
                init = self.array_default(elem, sizes)
        elif init is None:
            init = "''" if strdim is not None else self.default(ty)
            if strdim is not None:
                init = '""'
        if static:
            py = f"{self.fn}__{name}"
            ent = (ent_type, py, True, strdim, False)
            self.scopes[-1][name] = ent
            self.gvars[py] = ent
            self.statics.append(f"{py} = {init}")
            return
        py0, glob = self.py(name), top or not self.scopes
        handle = n.lower().endswith(("handle_t", "_handle", "handle"))  # out-param handles need a cell even with no initialiser
        cell = (self.want_cell(py0, glob, ent_type) and not dims and strdim is None
                and (handle or not init.startswith("_Dummy(")) and n not in self.structs)
        ent = self.declare(name, ent_type, strdim, top, cell)
        self.emit(f"{ent[1]} = _Cell({init})" if cell else f"{ent[1]} = {init}")

    # ---- statements ----------------------------------------------------------------------------------
    def block_or_stmt(self):
        self.ind += 1
        n0 = len(self.out)
        self.statement()
        if len(self.out) == n0:
            self.emit("pass")
        self.ind -= 1

    def cond(self):
        self.expect("(")
        e = self.expr()
        self.expect(")")
        c = e.c
        return c[1:-1] if c.startswith("(") and c.endswith(")") and c.count("(") == 1 + c[1:-1].count("(") and \
            self._balanced(c[1:-1]) else c

    @staticmethod
    def _balanced(s):
        d = 0
        for ch in s:
            d += ch == "("
            d -= ch == ")"
            if d < 0:
                return False
        return d == 0

    def statement(self):
        t = self.peek()
        k, v = t[0], t[1]
        if k == "op" and v == "{":
            self.i += 1
            self.scopes.append({})
            while not self.is_op("}"):
                if self.peek()[0] == "eof":
                    self.err("missing '}'")
                self.statement()
            self.i += 1
            self.scopes.pop()
            return
        if k == "op" and v == ";":
            self.i += 1
            return
        if k == "id":
            if v == "if":
                self.i += 1
                return self.s_if("if")
            if v == "while":
                self.i += 1
                c = self.cond()
                self.emit(f"while {c}:")
                self.loops.append(("loop", []))
                self.block_or_stmt()
                self.loops.pop()
                return
            if v == "do":
                return self.s_do()
            if v == "for":
                return self.s_for()
            if v == "switch":
                return self.s_switch()
            if v == "return":
                self.i += 1
                if self.accept(";"):
                    self.emit("return")
                else:
                    e = self.expr()
                    self.expect(";")
                    rt = self.fn_ret or UNK
                    self.emit(f"return {self.conv(e, rt)}")
                return
            if v == "break":
                self.i += 1
                self.expect(";")
                self.emit("break")
                return
            if v == "continue":
                self.i += 1
                self.expect(";")
                return self.do_continue()
            if v in ("goto", "try", "throw", "template", "namespace", "class"):
                raise CppError(f"'{v}' is not supported by the C++ translator", t[2])
            if v == "struct" and (self.peek(2)[1] == "{" or self.peek(1)[1] == "{"):
                return self.struct_def()
            if v == "enum":
                return self.enum_def()
            if v in ("typedef", "using"):
                return self.typedef()
            if self.is_decl():
                return self.declaration()
        e = self.expr(stmt=True)
        self.expect(";")
        if e.c and not re.fullmatch(r"[\d.]+|True|False|None", e.c):
            self.emit(e.c)

    def do_continue(self):
        for k in range(len(self.loops) - 1, -1, -1):
            kind, data = self.loops[k]
            if kind == "loop":
                for line in data:
                    self.emit(line)
                self.emit("continue")
                if k < len(self.loops) - 1:
                    pass
                return
            if kind == "switch":
                data["cont"] = True
                self.emit(f"_c{data['id']} = True")
                self.emit("break")
                return
        self.emit("continue")

    def s_if(self, kw):
        c = self.cond()
        self.emit(f"{kw} {c}:")
        self.block_or_stmt()
        if self.accept("else"):
            if self.is_id("if"):
                self.i += 1
                self.s_if("elif")
            else:
                self.emit("else:")
                self.block_or_stmt()

    def s_do(self):
        self.i += 1
        sid = self.new_id()
        self.emit(f"_d{sid} = True")
        hdr = len(self.out)
        self.emit("while True:")
        self.loops.append(("loop", []))
        self.ind += 1
        self.emit(f"_d{sid} = False")
        self.statement()
        self.ind -= 1
        self.loops.pop()
        self.expect("while")
        c = self.cond()
        self.expect(";")
        self.out[hdr] = "    " * self.ind + f"while _d{sid} or ({c}):"

    def for_part_stmts(self, end):
        stmts = []
        while not self.is_op(end):
            e = self.expr_nocomma(True)
            stmts.append(e.c)
            if not self.accept(","):
                break
        return stmts

    def expr_nocomma(self, stmt):
        return self.assign(stmt)

    def s_for(self):
        self.i += 1
        self.expect("(")
        self.scopes.append({})
        start = self.i
        init_lines = []
        saved = self.out
        self.out = []
        declared = None
        if not self.is_op(";"):
            if self.is_decl():
                before = set(self.scopes[-1])
                self.declaration()
                declared = [n for n in self.scopes[-1] if n not in before]
            else:
                for s in self.for_part_stmts(";"):
                    self.emit(s)
                self.expect(";")
        else:
            self.i += 1
        init_lines = [ln.strip() for ln in self.out]
        self.out = saved
        cond = V("True", BOOL)
        if not self.is_op(";"):
            cond = self.expr()
        self.expect(";")
        incs = self.for_part_stmts(")")
        self.expect(")")
        body_start = self.i
        rng = self.try_range(declared, init_lines, cond, incs)
        if rng:
            hdr, watch = rng
            mark = len(self.out)
            self.emit(hdr)
            self.loops.append(("loop", []))
            self.block_or_stmt()
            self.loops.pop()
            body = "\n".join(self.out[mark + 1:])
            hit = any(re.search(rf"(?<![\w.]){re.escape(self.py(w))}\s*(?:[-+*/%&|^]|<<|>>|:)?=(?!=)", body)
                      for w in watch)
            if not hit:
                self.scopes.pop()
                return
            del self.out[mark:]  # body writes to the loop variable/limit: use the general form
            self.i = body_start
        for ln in init_lines:
            self.emit(ln)
        self.emit(f"while {cond.c}:")
        self.loops.append(("loop", list(incs)))
        self.ind += 1
        n0 = len(self.out)
        self.statement()
        for s in incs:
            self.emit(s)
        if len(self.out) == n0:
            self.emit("pass")
        self.ind -= 1
        self.loops.pop()
        self.scopes.pop()

    def try_range(self, declared, init_lines, cond, incs):
        """for (int i = a; i < b; i++) -> range() when safe."""
        if not declared or len(declared) != 1 or len(init_lines) != 1 or len(incs) != 1:
            return None
        var = declared[0]
        py = self.py(var)
        m0 = re.fullmatch(rf"{re.escape(py)} = (.+)", init_lines[0].strip())
        m1 = re.fullmatch(rf"\(?{re.escape(py)} (<|<=|>|>=) ([\w.]+)\)?", cond.c)
        m2 = re.fullmatch(rf"{re.escape(py)} (\+|-)= (\d+)|{re.escape(py)} = \(\(?{re.escape(py)} \+ 1\)? & \d+\)", incs[0])
        if not (m0 and m1 and m2):
            return None
        step = (int(m2.group(2)) if m2.group(2) else 1) * (-1 if m2.group(1) == "-" else 1)
        op, lim = m1.group(1), m1.group(2)
        if (step > 0) != (op in ("<", "<=")):
            return None
        if not re.fullmatch(r"\d+|[A-Za-z_]\w*", lim):
            return None
        stop = lim if op == "<" or op == ">" else (f"{lim} + 1" if step > 0 else f"{lim} - 1")
        rg = f"range({m0.group(1)}, {stop}{'' if step == 1 else f', {step}'})"
        return f"for {py} in {rg}:", [var] + ([] if lim.isdigit() else [lim])

    def s_switch(self):
        self.i += 1
        self.expect("(")
        e = self.expr()
        self.expect(")")
        self.expect("{")
        sid = self.new_id()
        self.emit(f"_sw{sid} = {e.c}")
        self.emit(f"_m{sid} = False")
        self.emit(f"_c{sid} = False")
        cv_line = len(self.out)
        self.emit("")
        self.emit("while True:")
        ctx = {"id": sid, "cont": False}
        self.loops.append(("switch", ctx))
        self.ind += 1
        cases = []
        self.scopes.append({})
        while not self.is_op("}"):
            if self.accept("case"):
                v = self.ternary()
                self.expect(":")
                cases.append(v.c)
                self.emit(f"if _m{sid} or _sw{sid} == {v.c}:")
            elif self.accept("default"):
                self.expect(":")
                self.emit(f"if _m{sid} or _sw{sid} not in _cv{sid}:")
            else:
                self.err("expected 'case' or 'default'")
            self.ind += 1
            self.emit(f"_m{sid} = True")
            while not (self.is_id("case") or self.is_id("default") or self.is_op("}")):
                if self.peek()[0] == "eof":
                    self.err("missing '}' in switch")
                self.statement()
            self.ind -= 1
        self.scopes.pop()
        self.expect("}")
        self.emit("break")
        self.ind -= 1
        self.loops.pop()
        self.out[cv_line] = "    " * self.ind + f"_cv{sid} = ({', '.join(cases)}{',' if cases else ''})"
        if ctx["cont"]:
            self.emit(f"if _c{sid}:")
            self.ind += 1
            n0 = len(self.out)
            self.do_continue()
            self.ind -= 1

    # ---- top level -------------------------------------------------------------------------------------
    def struct_def(self):
        self.i += 1
        name = None
        if self.peek()[0] == "id":
            name = self.nxt()[1]
        self.expect("{")
        fields = []
        while not self.is_op("}"):
            base, _ = self.parse_type()
            while True:
                ty = base
                while self.accept("*"):
                    ty = (ty[0], ty[1] + 1)
                fname = self.nxt()[1]
                dims = []
                while self.accept("["):
                    dims.append(self.expr().c)
                    self.expect("]")
                fty = self.norm((ty[0], ty[1] + len(dims)))
                dflt = self.array_default((ty[0], ty[1]), dims) if dims and fty != STR else self.default(fty)
                fields.append((fname, fty, dflt))
                if not self.accept(","):
                    break
            self.expect(";")
        self.expect("}")
        if name is None and self.peek()[0] == "id":
            name = self.peek()[1]
        if name is None:
            self.err("anonymous struct needs a name")
        self.structs[name] = [(f, t) for f, t, _ in fields]
        names = ", ".join(repr(self.py(f)) for f, _, _ in fields)
        self.emit(f"class {name}:")
        self.emit(f"    __slots__ = ({names}{',' if len(fields) == 1 else ''})")
        params = ", ".join(f"{self.py(f)}=None" for f, _, _ in fields)
        self.emit(f"    def __init__(self{', ' if params else ''}{params}):")
        if not fields:
            self.emit("        pass")
        for f, _, d in fields:
            self.emit(f"        self.{self.py(f)} = {d} if {self.py(f)} is None else {self.py(f)}")
        if self.peek()[1] != ";":
            if self.peek()[0] == "id":  # typedef-less trailing declarator
                self.i -= 0
                self.finish_struct_vars(name)
                return
        self.expect(";")

    def finish_struct_vars(self, name):
        while self.peek()[0] == "id":
            vn = self.nxt()[1]
            self.finish_decl_simple(vn, (name, 0))
            if not self.accept(","):
                break
        self.expect(";")

    def finish_decl_simple(self, name, ty):
        ent = self.declare(name, ty, None, not self.scopes)
        self.emit(f"{ent[1]} = {self.default(ty)}")

    def enum_def(self):
        self.i += 1
        self.accept("class")
        self.accept("struct")
        name = None
        if self.peek()[0] == "id":
            name = self.nxt()[1]
        if self.accept(":"):
            self.parse_type()
        self.expect("{")
        prev = None
        while not self.is_op("}"):
            m = self.nxt()[1]
            if self.accept("="):
                e = self.ternary()
                code = e.c
            else:
                code = "0" if prev is None else f"({prev} + 1)"
            ent = self.declare(m, INT, None, not self.scopes)
            self.emit(f"{ent[1]} = {code}")
            prev = ent[1]
            if not self.accept(","):
                break
        self.expect("}")
        if name is None and self.peek()[0] == "id":
            self.typedefs[self.peek()[1]] = INT
            self.i += 1
        elif name:
            self.enums.add(name)
        self.accept(";")

    def typedef(self):
        kw = self.nxt()[1]
        if kw == "using":
            name = self.nxt()[1]
            self.expect("=")
            ty, _ = self.parse_type()
            self.typedefs[name] = self.norm(ty)
            self.expect(";")
            return
        if self.is_id("struct") and (self.peek(1)[1] == "{" or self.peek(2)[1] == "{"):
            j = self.i
            self.i += 1
            tag = self.peek()[1] if self.peek()[0] == "id" else None
            depth = 0
            k = self.i
            while True:
                if self.toks[k][1] == "{":
                    depth += 1
                if self.toks[k][1] == "}":
                    depth -= 1
                    if depth == 0:
                        break
                k += 1
            alias = self.toks[k + 1][1]
            if tag is None:
                self.toks[j + 1:j + 1] = [["id", alias, self.toks[j][2]]]
            self.i = j
            self.struct_def_typedef(alias)
            return
        if self.is_id("enum"):
            self.enum_def()
            return
        ty, _ = self.parse_type()
        if self.is_op("(") or self.is_op("*") and self.is_op("(", 1):
            self.skip_to_semicolon()
            return
        name = self.nxt()[1]
        self.typedefs[name] = self.norm(ty)
        self.expect(";")

    def struct_def_typedef(self, alias):
        self.struct_def_inner_alias = alias
        self.struct_def()
        if self.toks[self.i][0] == "id":
            self.i += 1
        self.accept(";")

    def skip_to_semicolon(self):
        while not self.is_op(";") and self.peek()[0] != "eof":
            self.i += 1
        self.i += 1

    def function(self, ret, name):
        ret = self.norm(ret)
        self.expect("(")
        params = []
        if not self.is_op(")"):
            while True:
                if self.is_id("void") and self.is_op(")", 1):
                    self.i += 1
                    break
                if self.is_op("..."):
                    self.err("variadic functions are not supported")
                pty, fl = self.parse_type()
                if "ref" in fl and pty[1] == 0 and pty[0] in INTNAMES | {"float"}:
                    self.warnings.append(f"line {self.peek()[2]}: reference parameter '{name}' is passed by value")
                if self.is_op("(") and self.is_op("*", 1):
                    self.i += 2
                    pn = self.nxt()[1]
                    self.expect(")")
                    self.skip_balanced()
                    pty = UNK
                else:
                    pn = self.nxt()[1] if self.peek()[0] == "id" else f"_p{len(params)}"
                dflt = None
                while self.accept("["):
                    if not self.is_op("]"):
                        self.expr()
                    self.expect("]")
                    pty = (pty[0], pty[1] + 1)
                pty = self.norm(pty)
                if self.accept("="):
                    dflt = self.conv(self.assign(), pty)
                params.append((pn, pty, dflt))
                if not self.accept(","):
                    break
        self.expect(")")
        self.funcs[name] = (ret, [p[1] for p in params])
        if self.accept(";"):
            return
        if not self.is_op("{"):
            self.err("expected function body")
        self.defined.add(name)
        self.fn, self.fn_ret = name, ret
        self.scopes = [{}]
        cells = []
        for pn, pty, _ in params:
            cell = self.want_cell(self.py(pn), False, pty)
            self.declare(pn, pty, cell=cell)
            if cell:
                cells.append(f"{self.py(pn)} = _Cell({self.py(pn)})")
        self.assigned = set()
        self.statics = []
        mark = len(self.out)
        self.ind = 1
        self.statement()
        self.ind = 0
        body = cells and ["    " + c for c in cells] + self.out[mark:] or self.out[mark:]
        del self.out[mark:]
        plist = ", ".join(f"{self.py(pn)}" + (f"={d}" if d is not None else "") for pn, _, d in params)
        for s in self.statics:
            self.emit(s)
        self.emit(f"def {self.py(name)}({plist}):")
        if self.assigned:
            self.emit(f"    global {', '.join(sorted(self.assigned))}")
        if not body:
            body = ["    pass"]
        self.out.extend(body)
        self.scopes = []
        self.fn = None
        self.fn_ret = None

    fn_ret = None

    def program(self):
        while self.peek()[0] != "eof":
            t = self.peek()
            if t[1] == ";" or t[1] == "}":
                self.i += 1
                continue
            if t[0] == "id":
                if t[1] == "extern" and self.peek(1)[0] == "str":
                    self.i += 2
                    if self.accept("{"):
                        continue
                    continue
                if t[1] in ("template", "namespace", "class", "union"):
                    raise CppError(f"'{t[1]}' is not supported by the C++ translator", t[2])
                if t[1] == "struct" and (self.peek(1)[1] == "{" or self.peek(2)[1] == "{"):
                    self.struct_def()
                    continue
                if t[1] == "enum" and (self.peek(1)[1] == "{" or self.peek(2)[1] == "{" or self.peek(3)[1] == "{"
                                       or self.peek(2)[1] == ":" or self.peek(3)[1] == ":"):
                    self.enum_def()
                    continue
                if t[1] in ("typedef", "using"):
                    self.typedef()
                    continue
            self.declaration(top=True)

    def run(self):
        for _ in range(2):
            self.reset_pass()
            self.program()
        return "\n".join(self.out) + "\n"


class Result:
    def __init__(self, code, tr, includes):
        self.code = code
        self.includes = includes
        self.unknown_calls = sorted(tr.unknown_calls - tr.defined)
        self.unknown_ids = sorted(tr.unknown_ids - tr.defined - set(tr.gvars) - set(tr.funcs))
        self.warnings = tr.warnings
        self.defined = tr.defined


def transpile(src, known=()):
    toks, includes = preprocess(src)
    tr = Translator(toks, known)
    code = tr.run()
    return Result(code, tr, includes)


def looks_like_cpp(src):
    return bool(re.search(r"\bvoid\s+(setup|loop|app_main)\s*\(\s*(void)?\s*\)", src)) and "def " not in src
