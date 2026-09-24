# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Tier 0: parse an operator utterance into typed frames, or decline to.

WHY THIS REPLACED THE KEYWORD LADDER
------------------------------------
The ladder -- the `intent_router` module plus each bridge's own
`IntentRouter.dispatch`, all deleted on 2026-09-22 -- was a first-match-wins
chain of `re.search` calls over bare keywords. Measured on 178 unique real
operator utterances exported from production on 2026-09-19:

    a rule fired                                54%
    ...but 70% of those hits explained under a quarter of the utterance
    false positives on non-command prose       100%  (9 of 9)
    utterances matching two rules at once       20%  (ladder ORDER decides)
    net, counting only hits that explain >=25%  ~16%

The failures are not near-misses, they are confident wrong actions:

    "stop what you are doing and wait until I tell you to continue"
        -> RESUME_RE matches the word `continue`, and RESUME_RE is first,
           so a STOP order RESUMES autonomy.
    "how many times have you had to stop for something in your way?"
        -> halts the robot.
    "put the block back on the grey table"  /  "Can you pick it back up?"
        -> both drive in REVERSE, on the word `back`.
    "how much charge is left in your battery?"
        -> spins in place, on the word `left`.

Every one of those is a keyword appearing somewhere in a sentence that means
something else. The defect is structural: **matching a substring is not
interpreting an utterance**, and a ladder that only counts hits cannot tell
the two apart.

THE FIVE THINGS THIS DOES DIFFERENTLY
-------------------------------------
1. **Clause structure before keywords.** The utterance is split into clauses,
   and a clause introduced by `until / when / after / once / unless / if /
   next time` is a TRIGGER, not an order to execute now. That single rule is
   what stops `...until I tell you to continue` from resuming.

2. **Question form is checked first.** `how many carts have you delivered?`
   is a QUERY. No amount of `stop`/`left`/`back` inside a question can reach a
   motor, because questions never produce a motion frame.

3. **A consumption gate.** A frame is only accepted if its matched span
   explains most of the clause once politeness is stripped. Matching `back`
   inside `put the block back on the grey table` explains 4 characters of 38,
   so it is rejected rather than executed.

4. **Specificity, not source order.** Every rule is tried and the LONGEST
   match wins, so adding a rule cannot silently shadow another one. The
   ladder's ordering hazard disappears with the ordering.

5. **It says what it could not do.** The result carries an intent class, the
   residue it failed to explain, and a human-readable reason. Declining is a
   first-class outcome: AMBIGUOUS (`put it over there` -- which `it`?) routes
   to a clarifying question, CONVERSATION routes to the model. Neither one
   pretends to be a command.

WHAT IT DELIBERATELY DOES NOT DO
--------------------------------
It does not answer. It returns frames; the bridge executes them. It does not
ground references -- resolving "the cart in spot 3" needs live state, so it
reports AMBIGUOUS with the phrase it could not resolve and lets the caller
resolve or ask. And it has no model in it: everything here is deterministic
and runs in tens of microseconds, so an offline bridge behaves identically to
a connected one.

USAGE
-----
    from omnisim_bridges.interpret import interpret, MOBILE

    r = interpret("turn left 90 degrees, then drive forward 1 metre", MOBILE)
    r.intent            # "command"
    [f.tool for f in r.frames]      # ["turn", "drive_forward"]
    r.frames[0].args                # {"angle_rad": 1.5707963267948966}

    interpret("how much charge is left in your battery?", MOBILE).intent
    # "query"  -- not a spin
"""
from __future__ import annotations

import math
import re
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

__all__ = [
    "interpret", "Interpretation", "Frame",
    "MOBILE", "ARM", "QUADRUPED", "DRONE", "ANY",
    "COMMAND", "QUERY", "DEFERRED", "CONSTRAINT", "MEMORY",
    "CONVERSATION", "AMBIGUOUS", "EMPTY", "CONDITIONAL", "evaluate_condition",
]

# ── Surfaces ────────────────────────────────────────────────────────
# Which bridge is asking. A rule only fires on a surface that can do it, so
# "sit" never reaches a wheeled tug and "drive forward" never reaches an arm.
MOBILE = "mobile"
ARM = "arm"
QUADRUPED = "quadruped"
DRONE = "drone"
ANY = "any"

# ── Intent classes ──────────────────────────────────────────────────
COMMAND = "command"            # execute now; frames are filled and valid
QUERY = "query"                # read-only question about state or history
DEFERRED = "deferred"          # do X when Y -- goes to intents.schedule
CONSTRAINT = "constraint"      # standing restriction -- intents.set_constraint
MEMORY = "memory"              # a fact or policy to remember
CONVERSATION = "conversation"  # only a model can answer this
AMBIGUOUS = "ambiguous"        # understood the verb, not the referent -> ask
EMPTY = "empty"
# "if your x is below 0.4 m, drive forward 0.7 m; otherwise stay put": a test
# on the robot's OWN measured pose. The parser reads it; the router evaluates
# it against a pose it measures, and runs the chosen branch through the gate.
CONDITIONAL = "conditional"

_NUM = r"[-+]?\d+(?:\.\d+)?"
# ⚠️ WHOLE WORDS, LONGEST FIRST. The angle unit used to be
# `(?:deg|degree|degrees|...)` with no boundary, so "degrees" matched as
# "deg" and the unmatched "rees" counted against the consumption gate.
# Whether a trailing direction was then noticed depended on the length of
# the word: "turn 35 degrees right" was declined and "rotate 45 degrees
# right" TURNED LEFT at confidence 0.95. Found 2026-09-23.
_LEN_UNIT = (r"(?:millimetres|millimeters|millimetre|millimeter|centimetres|"
             r"centimeters|centimetre|centimeter|metres|meters|metre|meter|"
             r"inches|inch|feet|foot|mm|cm|ft|m)\b")
_ANG_UNIT = r"(?:degrees|degree|degs|deg|radians|radian|rads|rad)\b"


@dataclass
class Frame:
    """One typed, validated call the bridge can execute."""
    tool: str
    args: Dict[str, Any] = field(default_factory=dict)
    span: Tuple[int, int] = (0, 0)
    source: str = ""            # the clause this came from
    rule: str = ""              # which rule produced it, for explainability

    def __repr__(self) -> str:  # pragma: no cover - debugging aid
        return f"Frame({self.tool}, {self.args})"


@dataclass
class Interpretation:
    """What tier 0 made of one utterance, including what it could not make."""
    intent: str
    frames: List[Frame] = field(default_factory=list)
    reason: str = ""
    residue: str = ""
    tier: int = 0
    confidence: float = 0.0
    trigger: Optional[str] = None   # DEFERRED / CONSTRAINT: the condition text
    # The utterance this came from. The safety gate needs it: its rules
    # are about the SENTENCE versus the frame, and it must work on
    # frames a model produced too, where there is no parse to inspect.
    text: str = ""
    # AMBIGUOUS only: a question the parser can put to the operator itself,
    # with no model and no motion ("How far should I drive?").
    ask: str = ""
    # CONDITIONAL only: a test on the robot's own measured pose, and the
    # frames for each outcome. Resolved by `route.resolve_conditional`.
    condition: Optional[Dict[str, Any]] = None
    then_frames: List[Frame] = field(default_factory=list)
    else_frames: List[Frame] = field(default_factory=list)

    @property
    def executable(self) -> bool:
        """True only for frames that may be dispatched without asking first."""
        return self.intent == COMMAND and bool(self.frames)

    def explain(self) -> str:
        """A sentence saying what happened. Free, because the parse knows."""
        if self.intent == COMMAND:
            calls = ", ".join(f"{f.tool}({_fmt(f.args)})" for f in self.frames)
            return f"parsed {len(self.frames)} command(s): {calls}"
        if self.residue:
            return f"{self.intent}: {self.reason} (unexplained: {self.residue!r})"
        return f"{self.intent}: {self.reason}"


def _fmt(args: Dict[str, Any]) -> str:
    return ", ".join(f"{k}={v!r}" for k, v in args.items())


# ── Normalisation ───────────────────────────────────────────────────
# Production text carries smart quotes, em dashes and mojibake from three
# different clients. Normalise once so no rule has to care.
_DASHES = dict.fromkeys(map(ord, "‐‑‒–—―−"), "-")
_QUOTES = {ord("‘"): "'", ord("’"): "'",
           ord("“"): '"', ord("”"): '"'}


# Spoken quantities. Without these, "drive forward two metres" matched the
# bare `drive forward` rule with no distance, and the executor's default put
# the robot 1 m down the floor -- a SILENT GUESS, which is worse than a
# refusal because nothing in the reply says a number was invented. Measured
# by commandbench unit-3/unit-4, where "half a metre" and "two metres" both
# travelled exactly 1.000 m.
_UNITS = {"zero": 0, "nought": 0, "one": 1, "two": 2, "three": 3,
          "four": 4, "five": 5, "six": 6, "seven": 7, "eight": 8, "nine": 9}
_TEENS = {"ten": 10, "eleven": 11, "twelve": 12, "thirteen": 13,
          "fourteen": 14, "fifteen": 15, "sixteen": 16, "seventeen": 17,
          "eighteen": 18, "nineteen": 19}
_TENS = {"twenty": 20, "thirty": 30, "forty": 40, "fifty": 50, "sixty": 60,
         "seventy": 70, "eighty": 80, "ninety": 90}
_SMALL = {**_UNITS, **_TEENS, **_TENS}
_U = "|".join(_UNITS)
_T = "|".join(_TEENS)
_X = "|".join(_TENS)
_BELOW_100 = rf"(?:(?:{_X})(?:[\s-]+(?:{_U}))?|{_T}|{_U})"
# ⚠️ ONE SPAN, NOT ONE WORD AT A TIME. Word-by-word replacement turned
# "sixty-five degrees" into "60-5 degrees", which no rule reads, and "one
# hundred and twenty" into "1 100 and 20".
_NUMBER_SPAN = (rf"(?:(?:a|{_U})[\s-]+hundred(?:[\s-]+(?:and[\s-]+)?"
                rf"{_BELOW_100})?|hundred|{_BELOW_100})")
_NUMBER_WORDS = re.compile(rf"\b{_NUMBER_SPAN}\b", re.IGNORECASE)
_DIGIT_WORDS = rf"(?:{_U}|oh)(?:\s+(?:{_U}|oh))*"
# "zero point seven", "one point two five". The digits after "point" must be
# spelled out, so "go to point 3, 4" keeps its coordinate.
_POINT = re.compile(
    rf"\b(?:(\d+)|({_NUMBER_SPAN}))\s+point\s+({_DIGIT_WORDS})\b",
    re.IGNORECASE)


def _word_value(span: str) -> int:
    total = cur = 0
    for tok in re.split(r"[\s-]+", span.lower()):
        if tok == "and" or not tok:
            continue
        if tok == "a":
            cur = cur or 1
        elif tok == "hundred":
            total += (cur or 1) * 100
            cur = 0
        else:
            cur += _SMALL[tok]
    return total + cur


def _point(m: re.Match) -> str:
    whole = m.group(1) or str(_word_value(m.group(2)))
    return f"{whole}.{_point_digits(m.group(3))}"


def _point_digits(words: str) -> str:
    return "".join("0" if t == "oh" else str(_UNITS[t])
                   for t in words.lower().split())


# Longest first, so "three quarters" is not eaten by "three".
_FRACTIONS = [
    ("three quarters of a", "0.75"), ("three quarters", "0.75"),
    ("a quarter of a", "0.25"), ("a quarter", "0.25"), ("quarter of a", "0.25"),
    ("a half of a", "0.5"), ("half of a", "0.5"), ("half a", "0.5"),
    ("a half", "0.5"), ("half", "0.5"),
]
_UNIT_WORDS = (r"(?=\s*(?:m|meter|metre|meters|metres|cm|centimetres?|"
               r"centimeters?|mm|millimetres?|millimeters?|foot|feet|inch|"
               r"inches|deg|degrees?|rad|radians?|turn|turns|"
               r"revolution|revolutions)\b)")
_DENOMINATORS = {"tenth": 10, "tenths": 10, "hundredth": 100,
                 "hundredths": 100, "quarter": 4, "quarters": 4,
                 "fifth": 5, "fifths": 5}
_OF_A_UNIT = re.compile(
    r"\b(\d+|an?)\s+(" + "|".join(_DENOMINATORS) + r")\s+of\s+(?:an?\s+|1\s+)?"
    + _UNIT_WORDS, re.IGNORECASE)
# ⚠️ Keyed on the WORDS "and a half", marked before the fractions pass turns
# "a half" into 0.5. Keying on "and 0.5" merged "between -0.5 and 0.5 m"
# into "-1 m" (found 2026-09-24).
_HALF_MARK = "and_a_half"
_AND_A_HALF = re.compile(
    r"\b(\d+(?:\.\d+)?)(\s*" + _LEN_UNIT[:-2] + r"|\s*" + _ANG_UNIT[:-2]
    + r")?\s+" + _HALF_MARK + r"\b", re.IGNORECASE)
# The same for quarters: "one and a quarter metres", "two and three quarters".
_PART_MARKS = {"and_a_quarter": 0.25, "and_three_quarters": 0.75}
_AND_A_PART = re.compile(
    r"\b(\d+(?:\.\d+)?)(\s*" + _LEN_UNIT[:-2] + r"|\s*" + _ANG_UNIT[:-2]
    + r")?\s+(and_a_quarter|and_three_quarters)\b", re.IGNORECASE)
# "one metre forty" means 1.40 m: a whole number of metres followed by a
# bare two-digit number of centimetres, and nothing that makes it a count.
_METRE_CENTIS = re.compile(
    r"\b(\d+)\s*(metres|meters|metre|meter|m)\s+(\d{2})\b"
    r"(?!\s*(?:%|centi|cm|milli|mm|deg|degrees?|rad|times|x\b|metres|meters|m\b))",
    re.IGNORECASE)
# "1 metre and 30 centimetres", "1 m 25 cm", "3 feet 6 inches": one distance.
_MIXED_LENGTH = re.compile(
    r"\b(\d+(?:\.\d+)?)\s*(metres|meters|metre|meter|m|feet|foot|ft)\s+"
    r"(?:and\s+)?(\d+(?:\.\d+)?)\s*(centimetres|centimeters|centimetre|"
    r"centimeter|cm|millimetres|millimeters|millimetre|millimeter|mm|inches|"
    r"inch)\b", re.IGNORECASE)


# "pi/4 radians", "π/2 rad", "2 pi radians": only before a radian unit, so a
# stray "pi" elsewhere is left alone.
_PI_RADIANS = re.compile(
    r"(?:\b(\d+(?:\.\d+)?)\s*\*?\s*)?(?:\bpi\b|π)(?:\s*/\s*(\d+(?:\.\d+)?))?"
    r"(?=\s*(?:radians|radian|rads|rad)\b)", re.IGNORECASE)


def _pi(m: re.Match) -> str:
    return f"{float(m.group(1) or 1) * math.pi / float(m.group(2) or 1):.6f}"


def _digits(s: str) -> str:
    s = s.replace("°", " degrees ")
    s = _PI_RADIANS.sub(_pi, s)
    s = re.sub(r"\band\s+a\s+half\b", _HALF_MARK, s, flags=re.I)
    s = re.sub(r"\band\s+a\s+quarter\b", "and_a_quarter", s, flags=re.I)
    s = re.sub(r"\band\s+three\s+quarters\b", "and_three_quarters", s, flags=re.I)
    for phrase, value in _FRACTIONS:
        s = re.sub(r"\b" + re.escape(phrase) + r"\b", value, s, flags=re.I)
    s = _POINT.sub(_point, s)
    # A bare "point five" only before a unit: "at this point two robots..."
    # is not a number.
    s = re.sub(r"\bpoint\s+(" + _DIGIT_WORDS + r")\b" + _UNIT_WORDS,
               lambda m: "0." + _point_digits(m.group(1)), s, flags=re.I)
    s = _NUMBER_WORDS.sub(lambda m: str(_word_value(m.group(0))), s)
    # "a metre" / "an inch" mean one of them -- but only immediately before
    # a unit, so "a cart" is left alone.
    s = re.sub(r"\ban?\b\s+" + _UNIT_WORDS, "1 ", s, flags=re.I)
    # "seven tenths of a metre" -> "0.7 metre".
    s = _OF_A_UNIT.sub(
        lambda m: f"{(1 if m.group(1).lower() in ('a', 'an') else int(m.group(1))) / _DENOMINATORS[m.group(2).lower()]:g} ",
        s)
    # "one and a half metres" / "a metre and a half" -> "1.5 metres".
    s = _AND_A_HALF.sub(
        lambda m: f"{float(m.group(1)) + 0.5:g}{m.group(2) or ''}", s)
    s = _AND_A_PART.sub(
        lambda m: f"{float(m.group(1)) + _PART_MARKS[m.group(3).lower()]:g}{m.group(2) or ''}", s)
    s = s.replace(_HALF_MARK, "and a half")      # one that joined nothing
    s = s.replace("and_a_quarter", "and a quarter").replace(
        "and_three_quarters", "and three quarters")
    s = _METRE_CENTIS.sub(
        lambda m: f"{int(m.group(1)) + int(m.group(3)) / 100:g} metres", s)
    s = _MIXED_LENGTH.sub(
        lambda m: f"{_metres(m.group(1), m.group(2)) + _metres(m.group(3), m.group(4)):g} metres",
        s)
    return s


# ── Typos, conservatively ───────────────────────────────────────────
# commandbench noise-1/noise-2: "drive foward 1 metre" and "drv fwd 1m"
# were DECLINED. That is the safe failure, but it is still a failure -- an
# operator typing in a hurry is the normal case, not an edge case.
#
# Fuzzy matching is also how a keyword ladder gets its false positives
# back, so this is deliberately narrow:
#   * a closed vocabulary of command words, nothing else;
#   * a token is only corrected if it is NOT already a word we know;
#   * edit distance 1, and only when EXACTLY ONE vocabulary word is that
#     close -- an ambiguous correction is no correction.
# "lift" is in the vocabulary precisely so it is never "corrected" to
# "left", which is one edit away and means something very different.
_VOCAB = {
    "drive", "forward", "forwards", "ahead", "backward", "backwards",
    "reverse", "turn", "rotate", "spin", "left", "right", "around",
    "stop", "halt", "freeze", "pause", "wait", "resume", "continue",
    "metre", "metres", "meter", "meters", "degree", "degrees",
    "centimetre", "centimetres", "radian", "radians",
    "wave", "stand", "sit", "walk", "pick", "place", "lift", "grab",
    "open", "close", "gripper", "hover", "land", "takeoff", "climb",
    "home", "cart", "trolley", "block", "cube", "table",
    "advance", "travel", "proceed", "creep", "pivot", "swivel", "retreat",
    "clockwise", "anticlockwise", "counterclockwise", "twice", "times",
    "millimetre", "millimetres", "feet", "inches",
}
_ABBREV = {
    "drv": "drive", "fwd": "forward", "fward": "forward", "bwd": "backward",
    "bck": "back", "rev": "reverse", "lft": "left", "rgt": "right",
    "rt": "right", "deg": "degrees", "degs": "degrees", "mtr": "metre",
    "mtrs": "metres", "cmd": "command",
}
# ⚠️ Inflections of vocabulary words, which must never be "corrected" INTO
# the base form. "drove" is one edit from "drive", and rewriting it turned
# "you already DROVE forward 2 metres, didn't you?" -- an accusation about
# the past -- into a present-tense order, which the robot obeyed.
_PROTECTED = {
    "have",  # auxiliary in "when you have finished", not a typo for wave
    "drove", "driven", "driving", "turned", "turning", "stopped", "stopping",
    "waved", "waving", "walked", "walking", "picked", "picking", "placed",
    "placing", "opened", "opening", "closed", "closing", "landed", "landing",
    "lifted", "lifting", "grabbed", "grabbing", "moved", "moving", "stood",
    "standing", "sat", "sitting", "went", "gone", "spun", "spinning",
    "climbed", "climbing", "paused", "pausing", "waited", "waiting",
    "resumed", "resuming", "halted", "hovered", "hovering", "reversed",
    # verbs one edit from a vocabulary word: "head" became "ahead" and
    # "come" became "home", so "come forward 30 cm" parsed as nothing.
    "head", "come", "inch", "scoot",
    # "three times over" became "three times hover", "cover 0.9 m" "hover".
    "over", "cover", "covered", "covering",
}


def _within_one_edit(a: str, b: str) -> bool:
    """True when a and b differ by one insert, delete or substitution."""
    la, lb = len(a), len(b)
    if abs(la - lb) > 1:
        return False
    if la == lb:
        return sum(1 for x, y in zip(a, b) if x != y) == 1
    if la > lb:
        a, b, la, lb = b, a, lb, la
    i = 0
    while i < la and a[i] == b[i]:
        i += 1
    return a[i:] == b[i + 1:]


def _despell(s: str) -> str:
    out = []
    for token in re.split(r"(\W+)", s):
        low = token.lower()
        if not low.isalpha() or low in _VOCAB or low in _PROTECTED:
            out.append(token)
            continue
        if low in _ABBREV:
            out.append(_ABBREV[low])
            continue
        if len(low) >= 4:
            near = [w for w in _VOCAB if _within_one_edit(low, w)]
            if len(near) == 1:          # ambiguous means leave it alone
                out.append(near[0])
                continue
        out.append(token)
    return "".join(out)


def _normalise(text: str) -> str:
    s = (text or "").translate(_DASHES).translate(_QUOTES)
    s = s.replace("�", " ")          # replacement char from bad decodes
    s = _digits(s)
    s = _despell(s)
    return re.sub(r"\s+", " ", s).strip()


# Politeness and discourse padding. Stripped only for the CONSUMPTION test --
# never from the text a rule matches against, so "can you stop" still parses.
#
# The adverbial tails matter as much as the politeness: "stop for a second"
# is a stop, and counting "for a second" against the gate is what made the
# first draft reject it. None of these change WHICH tool is called; they only
# stop the denominator from punishing ordinary English.
_FILLER = re.compile(
    r"\b(please|kindly|could you|can you|would you|will you|"
    r"i want you to|i need you to|i'd like you to|go ahead and|go and|"
    r"for me|right now|just now|just|now|ok|okay|alright|"
    r"actually|basically|maybe|perhaps|hey|hi|thanks|thank you|"
    r"at the moment|at this exact moment|if you can|when you can|"
    # adverbial tails
    r"for a (?:second|moment|minute|bit|while)|for now|a (?:second|minute)|"
    r"immediately|at once|straight away|in place|right there|over here|"
    r"with your work|with the work|with it|with that|as normal|as before|"
    r"and hold(?: there| position)?|steady|slowly|carefully|right away|"
    r"i'?d like|i would like|let'?s|let us|"
    # sequence markers: they order the steps, they add none
    r"first|firstly|next|finally|lastly|afterwards|worth|"
    r"if you would|if you could|if you don'?t mind|if possible|"
    # goal restatements: the steps already say it. Executing the literal
    # steps is what a model does with these too.
    r"(?:still\s+)?(?:pointing|facing)\s+the\s+same\s+(?:way|direction)|"
    r"keeping\s+(?:the\s+same|your)\s+(?:direction|orientation)|"
    r"to\s+(?:return\s+to|get\s+back\s+to|end\s+up\s+at|finish\s+at)\s+(?:your|the)\s+"
    r"(?:start|starting\s+(?:point|position)|original\s+(?:heading|position|spot))|"
    r"to\s+end\s+up\s+where\s+you\s+started|"
    r"(?:so\s+(?:that\s+)?you(?:'re|\s+are)?\s+)?facing\s+(?:your|the)\s+original\s+"
    r"(?:heading|direction)(?:\s+again)?|"
    r"to\s+face\s+(?:your|the)\s+original\s+(?:heading|direction)|"
    # manner that restates what the primitive already does: a drive keeps
    # its heading and a turn stays on the spot.
    r"keeping (?:the same|your(?: current)?|a constant|your present) heading|"
    r"in a straight line|without translating|on the spot|"
    r"again|one more time|the same|exactly|"
    # locatives that add no slot: they mean "from wherever you are"
    r"from your current location|from where you are(?: now)?|"
    r"where you are(?: now)?|from here|in front of you)\b",
    re.IGNORECASE)

# "until I tell you" / "until I say" is not a trigger to schedule -- it is an
# INDEFINITE HOLD, which intents.py already implements precisely because the
# 60 s quiet window auto-resumed and walked the tug back into a forbidden
# cell. Recognising it here is what turns the ladder's worst inversion into
# the correct pair of actions: stop NOW, and do not self-resume.
_HOLD_TRIGGER = re.compile(
    r"\b(?:until|till)\s+(?:i|you)\s+"
    r"(?:tell|say|give|release|let|signal|come back|get back)",
    re.IGNORECASE)


def _core(clause: str) -> str:
    """The clause with padding removed -- the denominator of the gate."""
    s = _FILLER.sub(" ", clause)
    s = re.sub(r"[^\w\s.-]", " ", s)
    return re.sub(r"\s+", " ", s).strip()


# ── Clause splitting ────────────────────────────────────────────────
# Coordinates contain commas ("drive to x 10.3, y -3.0"), so they are masked
# before splitting and restored after. Getting this wrong turns one command
# into two broken halves, which is how "y -3.0" becomes an unparseable clause.
_COORD = re.compile(
    r"\b(?:x\s*[=:]?\s*)?(" + _NUM + r")\s*,\s*(?:y\s*[=:]?\s*)?(" + _NUM + r")\b",
    re.IGNORECASE)
_SPLIT = re.compile(
    r"\s*(?:,|;|\.|:|-{1,2}\s|\band then\b|\bthen\b|\bafter that\b|"
    r"\bfollowed by\b|\band afterwards\b|\bafterwards\b|\band\b)\s+",
    re.IGNORECASE)


def _clauses(text: str) -> List[str]:
    masked: List[str] = []

    def _mask(m: re.Match) -> str:
        masked.append(m.group(0))
        return f"\x00{len(masked) - 1}\x00"

    held = _COORD.sub(_mask, text)
    parts = [p.strip(" .") for p in _SPLIT.split(held)]

    def _restore(s: str) -> str:
        return re.sub(r"\x00(\d+)\x00", lambda m: masked[int(m.group(1))], s)

    return [_restore(p) for p in parts if p.strip()]


# ── Guards, applied before any rule ─────────────────────────────────
# Order matters here and only here, and each guard is a claim about the whole
# utterance rather than about a keyword inside it.

# A leading modal + "you" is a polite imperative ("can you stop"), NOT a
# question -- the single most common way a real operator phrases an order.
# ⚠️ A modal is polite only with "you". "you" was optional, so "would a
# clockwise rotation of 50 degrees help?" read as a polite order and TURNED.
_POLITE_IMPERATIVE = re.compile(
    r"^\s*(?:so\s+)?(?:(?:can|could|would|will)\s+you\s+(?:please\s+)?|"
    r"please\s+)", re.IGNORECASE)

# "do" heads a question ("do you see it?") but not an imperative with an
# object ("do this 3 times: ...", "do a 90-degree turn").
_QUESTION_HEAD = re.compile(
    r"^\s*(?:so\s+|and\s+|but\s+)?"
    r"(what|where|when|why|how|which|who|whose|"
    r"is|are|was|were|do(?!\s+(?:this|that|these|the following|a|an|it)\b)|"
    r"does|did|have|has|had|am|"
    r"can|could|will|would|should|any)\b",
    re.IGNORECASE)

_AGGREGATE = re.compile(r"\b(how many|how much|how often|how long|"
                        r"count|total|in total|so far|altogether)\b", re.IGNORECASE)

# Imperative in form, question in effect. "tell me where you are" asks; it
# does not move. The ladder had no way to express the difference.
_TELL_ME = re.compile(
    r"^\s*(?:tell|show|give|let|remind)\s+(?:me|us)\b|"
    r"^\s*(?:confirm|report|describe|list|state)\b|"
    r"^\s*(?:status(?:\s+check)?|sitrep|telemetry)\b",
    re.IGNORECASE)

# Subordinators introduce a TRIGGER. Everything after one is a condition, and
# an imperative inside a condition must never be executed now. This is the
# rule that fixes the RESUME_RE inversion.
_SUBORDINATOR = re.compile(
    r"\b(until|till|when|whenever|once|after|before|unless|if|"
    r"as soon as|next time|while)\b", re.IGNORECASE)

_PROHIBITION = re.compile(
    r"\b(don't|do not|never|must not|mustn't|no longer|stay out of|"
    r"keep out of|steer clear of|avoid)\b", re.IGNORECASE)

# ── Guards added after commandbench measured where the parser ACTED ──
# when it should have held still. Every one of these was a case where the
# robot moved on a sentence that, read whole, was not an order.

# A retraction cancels what came before it. "drive forward 2 metres - no
# wait, make it 1" travelled 2 m (the head won) and would have travelled 3
# if both had run. The tail governs.
_RETRACTION = re.compile(
    r"\b(?:no wait|wait no|actually|"
    r"forget (?:that|it)|scratch that|belay that|ignore that|"
    r"i meant|make it|on second thought(?:s)?|instead)\b",
    re.IGNORECASE)

# ...and a retraction whose tail is a cancellation means DO NOTHING.
_CANCEL_TAIL = re.compile(
    r"\b(?:stay (?:where you are|put)|do nothing|never ?mind|"
    r"hold (?:still|position)|don't|do not|forget it|as you were)\b",
    re.IGNORECASE)

# ── Repetition: "N times" applies to a sequence the parser must also read ──
# Only the forms whose TOTAL is unambiguous. "twice: A, then B", "do this 3
# times: A, B", "2 repetitions of A, then B", "A, then B, twice". NOT "A,
# then repeat that twice": English splits on whether that is two runs or
# three, and a guess is a whole extra lap of motion. Those go to the model.
_REP_COUNT = r"(?:(?P<n>\d+)\s*(?:times|x)\b|(?P<word>twice|thrice)\b)"
_REP_PREFIX = re.compile(
    r"^\s*(?:(?:do|perform|repeat|run|execute|carry out|loop|cycle through|"
    r"go through|run through)\s+)?"
    r"(?:(?:this|that|it|the following(?: steps)?|these steps|the (?:pair|"
    r"sequence|pattern|routine|loop)|this (?:pair|sequence|pattern|routine|loop))\s+)?"
    + _REP_COUNT + r"(?:\s+over)?\s*[:,-]\s*(?P<body>.+)$", re.IGNORECASE)
_REP_OF = re.compile(
    r"^\s*(?:(?:do|perform|execute|run|carry out)\s+)?(?P<n>\d+)\s+"
    r"(?:repetitions|reps|rounds|cycles|iterations)\s+of\b\s*"
    r"(?:(?:this|the following|that|the)\b\s*)?"
    r"(?:(?:pair|sequence|pattern|routine|steps|moves)\b\s*)?[:,-]?\s*"
    r"(?P<body>.+)$", re.IGNORECASE)
_REP_SUFFIX = re.compile(
    r"^(?P<body>.+?)(?P<sep>\s*[,;:-]\s*|\s+)"
    r"(?:(?:and\s+)?do\s+(?:that|this|it|all (?:of )?that|the whole thing)\s+)?"
    + _REP_COUNT + r"(?:\s+(?:in total|in all|altogether|total|over))?\s*[.!]?\s*$",
    re.IGNORECASE)
_MAX_REPEAT = 10
_MAX_REPEATED_FRAMES = 16


def _repetition(text: str) -> Optional[Tuple[int, str, bool]]:
    """-> (count, body, applies_to_whole_body) or None when there is no count."""
    for rx, whole in ((_REP_PREFIX, True), (_REP_OF, True)):
        m = rx.match(text)
        if m:
            word = (m.groupdict().get("word") or "").lower()
            n = 2 if word == "twice" else 3 if word == "thrice" else int(m.group("n"))
            return n, m.group("body").strip(), whole
    m = _REP_SUFFIX.match(text)
    if m:
        word = (m.group("word") or "").lower()
        n = 2 if word == "twice" else 3 if word == "thrice" else int(m.group("n"))
        # "A, then B twice" could mean B twice. Only a separator, or an
        # explicit "do that", carries the count over the whole sequence.
        whole = bool(m.group("sep").strip()) or " do " in f" {m.group(0).lower()} "
        return n, m.group("body").strip(), whole
    return None


# A bare quantity in the tail re-fills the head's slot: "make it 1 metre".
_REQUANTIFY = re.compile(
    r"(?:make it|i meant|instead)?\s*(" + _NUM + r")\s*"
    r"(" + _LEN_UNIT + r"|" + _ANG_UNIT + r")\b", re.IGNORECASE)

# Asking whether an action is POSSIBLE is not ordering it. Only explicit
# markers count: "can you stop?" must stay an order.
_HYPOTHETICAL = re.compile(
    r"\b(?:in principle|in theory|hypothetically|theoretically|"
    r"if i (?:asked|told|said|wanted)|would you be able|are you able|"
    r"could you ever|is it possible)\b", re.IGNORECASE)

# An order that negates itself. "drive forward 1 metre without moving" is
# not a command with a caveat; it is incoherent, and guessing is wrong.
_SELF_NEGATING = re.compile(
    r"\bwithout\s+(?:moving|driving|turning|going|rotating|budging)\b|"
    r"\bbut\s+(?:don't|do not|never)\b|\bwhile\s+not\s+moving\b",
    re.IGNORECASE)

# "in ten minutes, turn left" is a schedule, not an order for now.
_DELAY = re.compile(
    r"\bin\s+(" + _NUM + r")\s*"
    r"(seconds?|secs?|minutes?|mins?|hours?|hrs?)\b", re.IGNORECASE)

# A destination the parser cannot resolve without remembering the past.
_PAST_PLACE = re.compile(
    r"\b(?:where you (?:started|began|were)|your (?:starting|original) "
    r"(?:point|position|pose|spot)|back to the start)\b", re.IGNORECASE)

# A magnitude no wheeled tug in a room-sized world was plausibly asked for.
# This is a SANITY rail, not a world bound -- the parser does not know the
# arena. commandbench bounds-1 drove 14.4 m on "500 metres" and left it.
# ⚠️ The real fix is a world-aware clamp in the bridge, which does not exist.
MAX_PLAUSIBLE_M = 50.0
MAX_PLAUSIBLE_RAD = 8 * math.pi      # four full turns

_MEMORY_CUE = re.compile(
    r"\b(remember (?:this|that)?|note that|for the (?:whole )?(?:site|shift|day)|"
    r"from now on|for future reference|keep in mind|important for)\b",
    re.IGNORECASE)

# Social pressure and honesty probes. These MUST reach the model: they are
# challenges to the robot's own account of itself, and any deterministic
# answer would be a fabrication.
_SOCIAL = re.compile(
    r"\b(admit (?:it|you)|you'?re (?:making that up|lying)|that is wrong|"
    r"that's wrong|you never |i was watching|i have (?:the )?cctv|"
    r"apologi[sz]e|your (?:log|count) is (?:a )?(?:known )?(?:bug|incorrect|wrong)|"
    # ⚠️ NO trailing \b. These alternatives end in a literal "?", and a word
    # boundary after "?" at end-of-string cannot match -- so the tag-question
    # alternatives never fired. Silent since they were written; exposed when
    # typo correction turned "you already DROVE forward" into a command and
    # the robot drove 2 m answering an accusation.
    r"are you sure|aren'?t you\?|haven'?t you\?|didn'?t you\?|wasn'?t it\?)",
    re.IGNORECASE)

# Bare pronouns with no antecedent in the utterance. The verb is understood,
# the object is not -- which is a question to ask, not a motion to guess.
_DEIXIS = {"it", "that", "this", "them", "those", "these", "there", "here",
           "the other side", "over there"}


@dataclass(frozen=True)
class Rule:
    """One parse rule: a pattern that must match, and the slots it fills."""
    tool: str
    pattern: re.Pattern
    build: Callable[[re.Match], Dict[str, Any]]
    surfaces: Tuple[str, ...] = (ANY,)
    name: str = ""

    def applies(self, surface: str) -> bool:
        # surface=ANY means "this caller can do anything" -- used by the
        # offline evaluation, and by a relay fronting several robots.
        return ANY in self.surfaces or surface == ANY or surface in self.surfaces


def _metres(value: str, unit: Optional[str]) -> float:
    v = float(value)
    u = (unit or "").lower()
    if u.startswith(("cm", "centi")):
        return v / 100.0
    if u.startswith(("mm", "milli")):
        return v / 1000.0
    if u in ("ft", "foot", "feet"):
        return v * 0.3048
    if u.startswith("inch"):
        return v * 0.0254
    return v


def _radians(value: str, unit: Optional[str]) -> float:
    v = float(value)
    if unit and unit.lower().startswith("rad"):
        return v
    return math.radians(v)


def _sign(verb: Optional[str], direction: Optional[str]) -> int:
    """Direction wins over verb: "drive BACK 2 m" reverses, "reverse 2 m" too."""
    token = (direction or verb or "").lower()
    return -1 if token.startswith(("back", "rev")) else 1


def _obj(raw: Optional[str], default: str = "") -> str:
    """Clean an object slot: "block back" -> "block", "the cube" -> "cube"."""
    s = (raw or default).strip().lower()
    s = re.sub(r"^(?:the|a|an|that|this)\s+", "", s)
    s = re.sub(r"\s+(?:back|again|up|down)$", "", s)
    return s.strip()


def _R(tool: str, pattern: str, build: Callable[[re.Match], Dict[str, Any]],
       surfaces: Sequence[str] = (ANY,), name: str = "") -> Rule:
    return Rule(tool, re.compile(pattern, re.IGNORECASE), build,
                tuple(surfaces), name or tool)


# ── Directions on EITHER side of the quantity ───────────────────────
# "drive 1 metre backwards" DROVE FORWARD at confidence 0.95: the rule read
# a direction only before the number, and the trailing one was too short to
# fail the consumption gate. A direction after the quantity is now read, a
# pair that disagrees ("advance 1 m backwards") is refused as a slot error,
# and `_parse_clause` refuses any direction word a rule left unread.
_FWD_WORDS = r"straight ahead|straight on|straight|forwards|forward|ahead"
_BWD_WORDS = (r"in reverse|to the rear|rearwards|rearward|backwards|backward|"
              r"reverse|back")
_DRIVE_VERBS = (r"drive|go|move|roll|reverse|back|advance|travel|proceed|"
                r"head|creep|inch|come|retreat|scoot|push|ease|pull|cover|"
                r"edge|trundle|send|nudge|shift")
# Words that sit between the order and its number without changing it.
_ADVERB = (r"(?:(?:just|exactly|precisely|about|around|roughly|approximately|"
           r"only|some)\s+)?")
# The robot as an explicit object: "spin the base ...", "send it ...".
_SELF_OBJECT = (r"(?:(?:the\s+(?:robot|base|chassis|platform|husky)|yourself|it|"
                r"your\s+(?:base|body|chassis|self))\s+)?")
_IN_PLACE = r"(?:(?:in\s+place|on\s+the\s+spot)\s+)?"
_PARTICIPLE = r"(?:(?:moving|going|heading|travelling|traveling|driving)\s+)?"
_LEFT_WORDS = (r"anti-?clockwise|counter-?clockwise|counter clockwise|"
               r"left|ccw")
_RIGHT_WORDS = r"clockwise|right|cw"
_TURN_DIRS = _LEFT_WORDS + "|" + _RIGHT_WORDS
_TURN_VERBS = r"turn|rotate|spin|yaw|pivot|swivel|swing"
# A trailing direction must END the clause (politeness aside), so "drive 2
# metres back to the dock" is not read as a reverse.
_END = (r"(?=(?:\s+(?:please|now|thanks|thank you|immediately|at once))*"
        r"\s*[.!?]*\s*$)")


def _drive_sign(word: Optional[str]) -> Optional[int]:
    w = (word or "").lower()
    if not w:
        return None
    if re.fullmatch(_BWD_WORDS + r"|retreat", w):
        return -1
    if re.fullmatch(_FWD_WORDS + r"|advance", w):
        return 1
    return None


def _turn_sign(word: Optional[str]) -> Optional[int]:
    w = (word or "").lower()
    if not w:
        return None
    return 1 if re.fullmatch(_LEFT_WORDS, w) else -1


def _agreed(signs: Sequence[Optional[int]]) -> int:
    """The one sign every stated direction agrees on; +1 when none is stated."""
    said = {s for s in signs if s is not None}
    if len(said) > 1:
        raise ValueError("the directions disagree")
    return said.pop() if said else 1


def _correction_sign(tail: str, plus: str, minus: str) -> Optional[int]:
    """The sign a correction's own direction words give, or None.

    Raises ValueError when the correction names both directions."""
    t = _core(tail)
    has_plus = re.search(r"\b(?:" + plus + r")\b", t, re.IGNORECASE)
    # Strip the plus words first: "counter-clockwise" contains "clockwise".
    t = re.sub(r"\b(?:" + plus + r")\b", " ", t, flags=re.IGNORECASE)
    has_minus = re.search(r"\b(?:" + minus + r")\b", t, re.IGNORECASE)
    if has_plus and has_minus:
        raise ValueError("the correction names both directions")
    return 1 if has_plus else -1 if has_minus else None


_COMMA_QUANTITY = re.compile(
    r"\b((?:" + _TURN_VERBS + r")\s+(?:to\s+(?:the|your)\s+)?(?:" + _TURN_DIRS + r")|"
    r"(?:" + _DRIVE_VERBS + r")\s+(?:" + _FWD_WORDS + "|" + _BWD_WORDS + r"))"
    r"\s*,\s*(?=[-+]?\d)", re.IGNORECASE)
_LEADING_DIR_COMMA = re.compile(
    r"^(\s*(?:" + _TURN_DIRS + r"))\s*,\s*(?=[-+]?\d)", re.IGNORECASE)
# More joins across a comma or colon that belong to ONE step:
#   "80 degrees, turn left"          quantity first, verb after
#   "Right turn: 60 degrees"         a named turn, then its size
#   "cover 0.9 metres, heading forward"  a distance, then its direction
_JOIN_ONE_STEP = [
    re.compile(r"(\d+(?:\.\d+)?\s*" + _ANG_UNIT[:-2] + r"\b(?:\s+(?:to\s+the\s+)?(?:"
               + _TURN_DIRS + r")\b)?)\s*,\s*(?=(?:" + _TURN_VERBS + r")\b)", re.IGNORECASE),
    re.compile(r"\b((?:" + _TURN_DIRS + r")[\s-]+(?:turn|rotation))\s*[:,]\s*(?=[-+]?\d)",
               re.IGNORECASE),
    re.compile(r"(\d+(?:\.\d+)?\s*" + _LEN_UNIT[:-2] + r")\s*,\s*(?=(?:moving|going|heading|"
               r"travelling|traveling|driving)\s+(?:" + _FWD_WORDS + "|" + _BWD_WORDS + r")\b)",
               re.IGNORECASE),
]


def _required_direction(m: re.Match) -> str:
    """The stated direction of a verbless or noun-form order; none is an error."""
    groups = m.groupdict()
    said = groups.get("pre") or groups.get("post")
    if not said:
        raise ValueError("no direction was stated")
    return said


def _needs_direction(m: re.Match, build: Callable[[re.Match], Dict[str, Any]]
                     ) -> Dict[str, Any]:
    _required_direction(m)
    return build(m)


def _drive_build(m: re.Match) -> Dict[str, Any]:
    g = m.groupdict()          # verbless rules have no `verb`, some no `post`
    sign = _agreed([_drive_sign(g.get("verb")), _drive_sign(g.get("pre")),
                    _drive_sign(g.get("post"))])
    return {"distance": _metres(m.group("n"), m.group("unit")) * sign}


def _turn_build(m: re.Match) -> Dict[str, Any]:
    sign = _agreed([_turn_sign(m.group("pre")), _turn_sign(m.group("post"))])
    return {"angle_rad": _radians(m.group("n"), m.group("unit")) * sign}


# ── The grammar ─────────────────────────────────────────────────────
# Every rule fills the slots its tool needs. Longest match wins, so ordering
# in this list carries no meaning -- which is the point.
RULES: List[Rule] = [
    # ---- motion, mobile --------------------------------------------
    _R("drive_to",
       r"\b(?:drive|go|move|head|navigate|take yourself|come)\s+(?:to|over to)?\s*"
       r"(?:the\s+)?(?:point\s+|position\s+|coordinates?\s+)?"
       r"x\s*[=:]?\s*(" + _NUM + r")\s*(?:,|\s+and)?\s*y\s*[=:]?\s*(" + _NUM + r")",
       lambda m: {"x": float(m.group(1)), "y": float(m.group(2))},
       [MOBILE], "drive_to_xy"),

    _R("drive_to",
       r"\b(?:drive|go|move|head|navigate)\s+to\s+(?:the\s+)?"
       r"(?:plant|point|spot|position)?\s*(?:at\s+)?"
       r"\(?\s*(" + _NUM + r")\s*,\s*(" + _NUM + r")\s*\)?",
       lambda m: {"x": float(m.group(1)), "y": float(m.group(2))},
       [MOBILE], "drive_to_pair"),

    _R("drive_forward",
       r"\b(?P<verb>" + _DRIVE_VERBS + r")\s*(?:up|off)?\s*" + _SELF_OBJECT +
       r"(?:(?P<pre>(?:" + _FWD_WORDS + "|" + _BWD_WORDS + r")\b)\s+)?"
       r"(?:by\s+|for\s+)?" + _ADVERB + r"(?P<n>" + _NUM + r")(?![\d.])\s*"
       r"(?P<unit>" + _LEN_UNIT + r")"
       r"(?:\s+" + _PARTICIPLE + r"(?P<post>(?:" + _FWD_WORDS + "|" + _BWD_WORDS + r")\b)"
       + _END + r")?",
       _drive_build, [MOBILE], "drive_n"),

    _R("drive_forward",
       r"\b(?:drive|go|move|roll)\s+(forward|forwards|ahead)\b(?!\s*" + _NUM + r")",
       lambda m: {"distance": None},
       [MOBILE], "drive_default"),

    # No number means no distance, never a 1 m default: `_parse_clause`
    # turns a drive with no distance into "How far should I back up?".
    _R("drive_forward",
       r"\b(?:reverse|back up|back off|drive back|go back|move back)\b"
       r"(?:\s+(?:by\s+)?(" + _NUM + r")\s*(" + _LEN_UNIT + r"))?",
       lambda m: {"distance": -_metres(m.group(1), m.group(2))
                  if m.group(1) else None},
       [MOBILE], "reverse"),

    # ⚠️ `(?!\s*<length unit>)`: "turn left 2 metres" turned 2 DEGREES.
    _R("turn",
       r"\b(?:" + _TURN_VERBS + r")\s*" + _SELF_OBJECT + _IN_PLACE +
       r"(?:(?:to\s+(?:the|your)\s+)?(?P<pre>(?:" + _TURN_DIRS + r")\b)\s*)?"
       r"(?:by\s+|about\s+|through\s+)?" + _ADVERB +
       r"(?P<n>" + _NUM + r")(?![\d.])\s*(?P<unit>" + _ANG_UNIT + r")?"
       r"(?!\s*" + _LEN_UNIT + r")"
       r"(?:\s*(?:to\s+(?:the|your)\s+)?(?P<post>(?:" + _TURN_DIRS + r")\b)" + _END + r")?",
       _turn_build, [MOBILE, QUADRUPED, DRONE], "turn_n"),

    # The noun forms: "a clockwise rotation of 50 degrees", "a 90-degree
    # turn to the right". Both demand the angle unit.
    _R("turn",
       r"\b(?:an?\s+)?(?:(?P<pre>(?:" + _TURN_DIRS + r")\b)\s+)?"
       r"(?:rotation|turn|spin|pivot)\s+(?:of\s+|by\s+|through\s+)?" + _ADVERB +
       r"(?P<n>" + _NUM + r")(?![\d.])\s*(?P<unit>" + _ANG_UNIT + r")"
       r"(?:\s*(?:to\s+(?:the|your)\s+)?(?P<post>(?:" + _TURN_DIRS + r")\b)" + _END + r")?",
       _turn_build, [MOBILE, QUADRUPED, DRONE], "turn_noun_of"),

    _R("turn",
       r"\b(?:an?\s+)?(?P<n>" + _NUM + r")(?![\d.])[\s-]*(?P<unit>" + _ANG_UNIT + r")"
       r"\s+(?:(?P<pre>(?:" + _TURN_DIRS + r")\b)\s+)?(?:rotation|turn|spin|pivot)"
       r"(?:\s+(?:to\s+(?:the|your)\s+)?(?P<post>(?:" + _TURN_DIRS + r")\b)" + _END + r")?",
       _turn_build, [MOBILE, QUADRUPED, DRONE], "turn_noun_pre"),

    # ---- coverage round 2 (2026-09-24) ---------------------------------
    # Verbless orders: "0.8 m forward", "back 45 cm", "70° to the left".
    # They MUST name a direction and explain the whole clause
    # (`_WHOLE_CLAUSE_RULES`), so "the charger is 2 metres forward of you"
    # is never an order. A turn also demands its unit ("left 90" declines).
    _R("drive_forward",
       r"^\s*(?:(?P<pre>(?:" + _FWD_WORDS + "|" + _BWD_WORDS + r")\b)\s+)?"
       r"(?P<n>" + _NUM + r")(?![\d.])\s*(?P<unit>" + _LEN_UNIT + r")"
       r"(?:\s+(?P<post>(?:" + _FWD_WORDS + "|" + _BWD_WORDS + r")\b)" + _END + r")?",
       lambda m: _needs_direction(m, _drive_build), [MOBILE], "drive_bare"),

    _R("drive_forward",
       r"^\s*(?P<pre>(?:" + _FWD_WORDS + "|" + _BWD_WORDS + r")\b)\s+"
       r"(?P<n>" + _NUM + r")(?![\d.])\s*(?P<unit>" + _LEN_UNIT + r")",
       _drive_build, [MOBILE], "drive_bare_pre"),

    _R("turn",
       r"^\s*(?:(?:to\s+(?:the|your)\s+)?(?P<pre>(?:" + _TURN_DIRS + r")\b)\s+)?"
       r"(?P<n>" + _NUM + r")(?![\d.])\s*(?P<unit>" + _ANG_UNIT + r")"
       r"(?:\s*(?:to\s+(?:the|your)\s+)?(?P<post>(?:" + _TURN_DIRS + r")\b)" + _END + r")?",
       lambda m: _needs_direction(m, _turn_build),
       [MOBILE, QUADRUPED, DRONE], "turn_bare"),

    # Quantity first, verb after: "80 degrees turn left", "35 degrees left
    # pivot" (the comma is joined away above). A direction is required.
    _R("turn",
       r"^\s*(?P<n>" + _NUM + r")(?![\d.])\s*(?P<unit>" + _ANG_UNIT + r")"
       r"(?:\s+(?:to\s+(?:the|your)\s+)?(?P<pre>(?:" + _TURN_DIRS + r")\b))?"
       r"\s+(?:" + _TURN_VERBS + r")\b"
       r"(?:\s+(?:to\s+(?:the|your)\s+)?(?P<post>(?:" + _TURN_DIRS + r")\b))?"
       r"(?:\s+now)?" + _END,
       lambda m: _needs_direction(m, _turn_build),
       [MOBILE, QUADRUPED, DRONE], "turn_qty_verb"),

    # A drive as a noun: "an 80 cm drive forward", "a 0.5 m reverse".
    _R("drive_forward",
       r"\b(?:an?\s+)?(?P<n>" + _NUM + r")(?![\d.])[\s-]*(?P<unit>" + _LEN_UNIT + r")"
       r"\s+(?:(?P<pre>(?:" + _FWD_WORDS + "|" + _BWD_WORDS + r")\b)\s+)?"
       r"(?P<verb>drive|run|roll|move|reverse|retreat)\b"
       r"(?:\s+(?P<post>(?:" + _FWD_WORDS + "|" + _BWD_WORDS + r")\b)" + _END + r")?",
       _drive_build, [MOBILE], "drive_noun"),

    # A fraction of a turn as a noun: "a quarter turn to the right", "half a
    # turn clockwise". A direction is required.
    _R("turn",
       r"\b(?:an?\s+)?(?P<n>" + _NUM + r")(?![\d.])\s*(?:turn|turns|revolution|revolutions)\b"
       r"(?:\s+(?:to\s+(?:the|your)\s+)?(?P<post>(?:" + _TURN_DIRS + r")\b)" + _END + r")?",
       lambda m: {"angle_rad": float(m.group("n")) * 2 * math.pi
                  * _agreed([_turn_sign(_required_direction(m))])},
       [MOBILE, QUADRUPED, DRONE], "turn_rev_noun"),

    # "a quarter turn" normalises to "0.25 turn", which the degree/radian
    # rule would read as 0.25 DEGREES. A turn is a unit of its own.
    _R("turn",
       r"\b(?:turn|rotate|spin)\s*(left|right|clockwise|anticlockwise)?\s*"
       r"(?:by\s+)?(" + _NUM + r")\s*(?:turn|turns|revolution|revolutions)\b",
       lambda m: {"angle_rad": float(m.group(2)) * 2 * math.pi
                  * (-1 if (m.group(1) or "").lower() in
                     ("right", "clockwise") else 1)},
       [MOBILE, QUADRUPED, DRONE], "turn_revolutions"),

    _R("turn",
       r"\b(?:turn|rotate|spin|swing)\s+(left|right)\b(?!\s*" + _NUM + r")",
       lambda m: {"angle_rad": math.pi / 2 *
                  (1 if m.group(1).lower() == "left" else -1)},
       [MOBILE, QUADRUPED, DRONE], "turn_dir"),

    _R("turn",
       r"\b(?:turn around|u-?turn|about face|spin around|turn back around)\b",
       lambda m: {"angle_rad": math.pi},
       [MOBILE, QUADRUPED, DRONE], "turn_around"),

    _R("set_velocity",
       r"\b(?:set\s+)?velocity\s+(" + _NUM + r")[,\s]+(" + _NUM + r")",
       lambda m: {"v": float(m.group(1)), "w": float(m.group(2))},
       [MOBILE], "set_velocity"),

    # ---- carts, mobile ---------------------------------------------
    _R("attach_trolley",
       r"\b(?:attach|hook up|hitch|couple|connect|pick up|collect|fetch|grab)\s+"
       r"(?:to\s+)?(?:the\s+)?"
       # An id may be the noun itself (TROLLEY_E) or follow it (cart B).
       # Splitting the noun from its suffix is what produced trolley='_E'.
       r"(?:((?:trolley|cart|wagon)[_-][A-Za-z0-9]+)|"
       r"(?:cart|trolley|wagon)"
       # Without this lookahead the id group swallows the next ordinary word:
       # "the cart out of spot 2" became trolley='out', and "the cart parked
       # in spot 3" became trolley='PARKED'.
       r"(?:\s+(?:called\s+|named\s+|id\s+)?"
       r"(?!(?:out|from|in|into|at|to|over|on|for|with|and|that|which|is|are|"
       r"parked|sitting|standing|waiting|next|back|here|there|now)\b)"
       r"([A-Za-z][A-Za-z0-9_]*))?)"
       # "...out of park spot 2" is a slot, not noise: without capturing it
       # the clause stays half-explained and the gate rejects the whole order.
       r"(?:\s+(?:out of|from|in|at)\s+(?:the\s+)?(?:park\s+)?"
       r"(?:spot|bay|slot|row)\s*(\d+))?",
       lambda m: {"trolley": (m.group(1) or m.group(2) or "").upper() or None,
                  "spot": int(m.group(3)) if m.group(3) else None},
       [MOBILE], "attach_trolley"),

    _R("detach_trolley",
       r"\b(?:detach|unhook|uncouple|drop|release|park|leave)\s+"
       r"(?:the\s+)?(?:cart|trolley|wagon)\b",
       lambda m: {}, [MOBILE], "detach_trolley"),

    # ---- arm --------------------------------------------------------
    # Three phrasings of the same act, because English puts the particle
    # wherever it likes: "pick up the cube" / "pick the cube up" / "pick it
    # back up". The old ladder read the third as DRIVE IN REVERSE.
    _R("pick",
       r"\bpick\s+up\s+(?:the\s+|that\s+|an?\s+)?([a-z][a-z0-9_ ]{1,28}?)"
       r"(?=\s*(?:$|,|\band\b|\boff\b|\bfrom\b|\bthen\b))|"
       r"\bpick\s+(?:the\s+|that\s+|an?\s+)?([a-z][a-z0-9_ ]{1,28}?)\s+up\b|"
       r"\b(?:pick|grasp|grab|lift)\s+(?:the\s+|that\s+|an?\s+)?"
       r"([a-z][a-z0-9_ ]{1,28}?)(?=\s*(?:$|,|\band\b|\boff\b|\bfrom\b|\bthen\b))",
       lambda m: {"object": _obj(m.group(1) or m.group(2) or m.group(3))},
       [ARM], "pick_object"),

    _R("place",
       r"\b(?:place|put|set|drop|move|stack|take|bring|deliver)\s+"
       r"(?:the\s+|it\s+|that\s+|an?\s+)?"
       r"([a-z][a-z0-9_ ]{1,28}?)?\s*"
       r"(?:on top of|onto|on|into|in|at|over to|over|to)\s+"
       r"(?:the\s+)?(over there|there|here|[a-z][a-z0-9_ ]{1,28})",
       lambda m: {"object": _obj(m.group(1), "it"),
                  "target": _obj(m.group(2))},
       [ARM, MOBILE], "place_on"),

    _R("open_gripper",
       r"\b(?:open|release|let go(?: of)?|ungrip)\s+(?:the\s+|your\s+)?"
       r"(?:gripper|grabber|grab bar|claw|hand|grip)\b",
       lambda m: {}, [ARM], "open_gripper"),

    _R("close_gripper",
       r"\b(?:close|shut|clamp|grip)\s+(?:the\s+|your\s+)?"
       r"(?:gripper|grabber|claw|hand|grip)\b",
       lambda m: {}, [ARM], "close_gripper"),

    _R("wave",
       r"\b(?:wave|say hello|greet|salute)"
       r"(?:\s+(?:hello|hi|goodbye|at me|to me|at the camera|to celebrate))?\b",
       lambda m: {}, [ARM, QUADRUPED], "wave"),

    # ---- quadruped ---------------------------------------------------
    _R("sit", r"\b(?:sit|sit down|lie down|crouch)\b", lambda m: {},
       [QUADRUPED], "sit"),
    _R("stand", r"\b(?:stand|stand up|get up|rise)\b", lambda m: {},
       [QUADRUPED], "stand"),
    _R("walk",
       r"\b(?:walk|trot|pace)\s*(?:forward|forwards|ahead)?\s*"
       r"(?:(" + _NUM + r")\s*(" + _LEN_UNIT + r"))?",
       lambda m: {"distance": _metres(m.group(1), m.group(2))
                  if m.group(1) else None},
       [QUADRUPED], "walk"),

    # ---- drone -------------------------------------------------------
    # `land` is this surface's `back`: an ordinary English word with a motor
    # behind it. "where did the package land?" must stay a question, which
    # the question guard settles before any rule here is tried.
    _R("takeoff",
       r"\b(?:take\s?off|takeoff|launch|lift\s?off)\b"
       r"(?:\s+(?:to|at))?\s*(?:an?\s+)?(?:altitude\s+(?:of\s+)?)?"
       r"(?:(" + _NUM + r")\s*(" + _LEN_UNIT + r")?)?",
       lambda m: {"altitude": _metres(m.group(1), m.group(2))
                  if m.group(1) else None},
       [DRONE], "takeoff"),

    _R("land",
       r"\b(?:land|touch\s?down|set\s?down)\b",
       lambda m: {}, [DRONE], "land"),

    _R("hover",
       r"\b(?:hover|station\s?keep|hold\s+(?:position|altitude))\b",
       lambda m: {}, [DRONE], "hover"),

    _R("move_body",
       r"\b(climb|ascend|go\s+up|rise|descend|drop\s+down|go\s+down|lower)\b"
       r"[^-\d]*(" + _NUM + r")\s*(" + _LEN_UNIT + r")?",
       lambda m: {"vertical": _metres(m.group(2), m.group(3))
                  * (-1 if m.group(1).lower().startswith(
                      ("desc", "drop", "go d", "lower")) else 1)},
       [DRONE], "climb"),

    _R("move_body",
       r"\b(?:fly|move|go)\s+(forward|forwards|ahead|back|backward|backwards)\b"
       r"[^-\d]*(" + _NUM + r")\s*(" + _LEN_UNIT + r")?",
       lambda m: {"forward": _metres(m.group(2), m.group(3))
                  * (-1 if m.group(1).lower().startswith("back") else 1)},
       [DRONE], "fly_forward"),

    # ---- universal ---------------------------------------------------
    # "stop what you're doing" must consume the whole clause, or the gate
    # rejects a plain stop order for being only four characters long.
    _R("stop",
       r"\b(?:stop|halt|freeze|brake|pause|cease|stand down)"
       r"(?:\s+(?:what\s+you(?:'re|\s+are)\s+doing|everything|all\s+work|"
       r"moving|driving|there|here))?\b|"
       r"\bhold\s+(?:still|position|there|on|fire)\b|"
       r"\bwait\b",
       lambda m: {}, [ANY], "stop"),

    _R("resume_autonomy",
       r"\b(?:resume|carry on|carrying on|keep (?:going|working|at it)|"
       r"continue|as you were|proceed|back to (?:work|it)|"
       r"get (?:back to|on) (?:work|it)|go back to (?:work|what you were)|"
       r"restart (?:your )?(?:work|loop|autonomy)|un-?pause)\b",
       lambda m: {}, [ANY], "resume"),

    _R("reset_to_home",
       r"\b(?:go home|return home|head home|reset to home|go back to (?:the )?start|"
       r"return to (?:the )?(?:start|dock|home)|dock yourself)\b",
       lambda m: {}, [ANY], "reset_to_home"),
]


# ── Query sub-classification ────────────────────────────────────────
# A query still deserves a routing decision: which read-only surface answers
# it. Getting this right is most of the remaining traffic.
_QUERY_KIND = [
    ("pose", re.compile(r"\b(where are you|your (?:x and y|position|pose|location)|"
                        r"where is the|where did|how far|"
                        # a drone's pose question is about height
                        r"altitude|how high|your height)\b", re.IGNORECASE)),
    ("activity", re.compile(r"\b(what (?:are|were) you (?:doing|up to)|"
                            r"what you are doing|what'?re you doing|"
                            r"current job|your (?:job|task|assignment)|"
                            r"status|sitrep|telemetry|report|"
                            r"what are you (?:working on|towing|carrying))\b",
                            re.IGNORECASE)),
    ("payload", re.compile(r"\b(carrying|towing|holding|hooked up|attached|"
                           r"got a cart)\b", re.IGNORECASE)),
    ("history", re.compile(r"\b(delivered|completed|finished|shipped|parked|"
                           r"last (?:three|two|few)|action history|"
                           r"this (?:session|shift))\b", re.IGNORECASE)),
    ("battery", re.compile(r"\b(charge|battery|power level)\b", re.IGNORECASE)),
    ("capability", re.compile(r"\b(can you reach|furthest point|could you "
                              r"actually|able to)\b", re.IGNORECASE)),
    ("scene", re.compile(r"\b(how many|is there|are there|anywhere|"
                         r"in the (?:cell|row|workspace))\b", re.IGNORECASE)),
    ("health", re.compile(r"\b(problems?|anomal|worried|okay|ok\b|"
                          r"keeping up|going well)\b", re.IGNORECASE)),
]


def _query_kind(text: str) -> str:
    for kind, rx in _QUERY_KIND:
        if rx.search(text):
            return kind
    return "state"


# ── The parser ──────────────────────────────────────────────────────
# The consumption gate. Below this share of the clause explained, a match is
# treated as a coincidence rather than an interpretation. 0.55 was chosen
# against the shipped router's own failures (`back` explains 0.11 of "put the
# block back on the grey table") and NOT tuned per-utterance on the corpus.
_GATE = 0.55

_DIRECTED_TOOLS = ("drive_forward", "turn", "walk", "move_body")
_ASK_DISTANCE = "needs a distance: "
# Verbless orders must explain (nearly) the whole clause.
_WHOLE_CLAUSE_RULES = ("drive_bare", "drive_bare_pre", "turn_bare")
# Noun phrases are orders only with nothing but a request verb around them.
_NOUN_RULES = ("turn_noun_of", "turn_noun_pre", "drive_noun", "turn_rev_noun")
_UNREAD_MEANING = re.compile(
    r"\b(?:left|right|clockwise|anti-?clockwise|counter-?clockwise|ccw|cw|"
    r"forwards?|ahead|backwards?|reverse|back|rearwards?|"
    r"twice|thrice|times|repeat|repetitions?|reps|rounds|laps|cycles)\b",
    re.IGNORECASE)
_UNREAD_MAGNITUDE = re.compile(
    r"\d|\b(?:" + _LEN_UNIT[3:-3] + "|" + _ANG_UNIT[3:-3] + r")\b",
    re.IGNORECASE)
_REQUEST_ONLY = re.compile(
    r"(?:(?:do|make|perform|execute|give me|give us|we need|i need|i want)"
    r"\s*)*", re.IGNORECASE)


def _best_match(clause: str, surface: str) -> Optional[Tuple[Rule, re.Match]]:
    """Longest match wins, so a new rule cannot shadow an existing one."""
    best: Optional[Tuple[Rule, re.Match]] = None
    for rule in RULES:
        if not rule.applies(surface):
            continue
        m = rule.pattern.search(clause)
        if m is None:
            continue
        if best is None or len(m.group(0)) > len(best[1].group(0)):
            best = (rule, m)
    return best


def _has_deixis(args: Dict[str, Any]) -> Optional[str]:
    for key, value in args.items():
        if isinstance(value, str) and value.strip().lower() in _DEIXIS:
            return f"{key}={value!r}"
    return None


def _parse_clause(clause: str, surface: str) -> Tuple[Optional[Frame], str, str]:
    """-> (frame, why_not, residue). Exactly one of frame / why_not is set."""
    # An imprecise quantity is still a missing distance. Keep this anchored
    # so a phrase inside a report/condition cannot become a new command.
    if surface in (MOBILE, ANY) and re.fullmatch(
            r"\s*(?:please\s+)?(?:" + _DRIVE_VERBS + r")\s*"
            r"(?:(?:up|off|" + _FWD_WORDS + '|' + _BWD_WORDS + r")\s+)?"
            r"(?:a\s+(?:little(?:\s+bit)?|bit|touch|tad|nudge)|slightly|"
            r"a\s+few\s+(?:metres|meters|centimetres|centimeters))"
            r"(?:\s+please)?\s*[.!?]*\s*", clause, re.I):
        return (None, _ASK_DISTANCE + 'How far should I move? Give me an explicit distance.', clause)
    found = _best_match(clause, surface)
    if found is None:
        return None, "no rule matched", clause
    rule, m = found

    core = _core(clause)
    matched = _core(m.group(0))
    share = len(matched) / max(1, len(core))
    if share < _GATE or (rule.name in _WHOLE_CLAUSE_RULES and share < 0.9):
        return (None,
                f"{rule.name} matched {matched!r} but that explains only "
                f"{share:.0%} of the clause",
                core[len(matched):].strip() or core)

    # ⚠️ A direction or a count the rule did not read changes the action, so
    # a high share is not enough. "move 50 cm back" explained 60% of its
    # clause and drove FORWARD; "turn left 90 degrees twice" turned once.
    if rule.tool in _DIRECTED_TOOLS:
        rest = _core(clause[:m.start()] + " " + clause[m.end():])
        # A number or a unit the rule did not read is a magnitude it did not
        # read. Found by the held-out harness comparison (2026-09-23), after
        # its freeze: "roll ahead one metre forty" parsed as 1.0 m with the
        # "40" left over, and "drive forward ___ m" as a drive with no
        # distance, which the executor fills with a 1 m default.
        unread = _UNREAD_MEANING.search(rest) or _UNREAD_MAGNITUDE.search(rest)
        if unread:
            return (None, f"{rule.name} left {unread.group(0)!r} unread",
                    rest)
        # A noun phrase is an order only when nothing but a request verb is
        # around it: "a clockwise rotation of 50 degrees, please" orders one,
        # "... would help" does not.
        if rule.name in _NOUN_RULES and not _REQUEST_ONLY.fullmatch(rest):
            return (None, f"{rule.name} is a noun phrase inside a sentence "
                          f"about something else", rest)

    try:
        args = rule.build(m)
    except (ValueError, IndexError, TypeError) as exc:   # slot did not convert
        return None, f"{rule.name} slots invalid: {exc}", clause

    args = {k: v for k, v in args.items() if v is not None}
    # ⚠️ OWNER DECISION 2026-09-23: A DRIVE WITH NO DISTANCE ASKS HOW FAR.
    # "drive forward", "go ahead" and "back up" used to reach the router with
    # no distance, and its adapter filled in 1 m: a magnitude nobody said,
    # which the gate's invented-magnitude rule could not see because the
    # default was applied after it. The drive is not emitted; the caller
    # gets a question instead (see `_interpret`).
    if rule.tool == "drive_forward" and "distance" not in args:
        direction = "back up" if rule.name == "reverse" else "drive"
        return (None, f"{_ASK_DISTANCE}How far should I {direction}? Give me "
                      f"a distance, like \"1 metre\".", clause)
    deictic = _has_deixis(args)
    if deictic:
        return None, f"{rule.name} needs a referent for {deictic}", clause

    sane = _implausible(args, clause)
    if sane:
        # ⚠️ Prefixed so the caller routes this to AMBIGUOUS, which the
        # executor ANSWERS. A low-confidence CONVERSATION returns None, and
        # the keyword ladder each bridge used to carry behind the parser
        # then drove the 500 m the parser had just refused: the decline was
        # right and the fallthrough undid it. The ladders were deleted on
        # 2026-09-22, but the principle outlives them and is why this is
        # still not a None: a safety refusal must never be a fallthrough.
        return None, f"needs confirmation: {sane}", clause

    return Frame(rule.tool, args, m.span(), clause, rule.name), "", ""


def _implausible(args: Dict[str, Any], clause: str) -> Optional[str]:
    """Reject a slot that parsed cleanly but cannot have been meant.

    commandbench bounds-1: "drive forward 500 metres" parsed perfectly and
    drove 14.4 m before the run ended, leaving the arena. A parser cannot
    know the world, but it can know that no operator means 500 m from a
    tug -- and that a direction and a sign should agree.
    """
    d = args.get("distance")
    if d is not None and abs(d) > MAX_PLAUSIBLE_M:
        return (f"{abs(d):.0f} m is far beyond one order's worth of travel; "
                f"confirm it, or give me a coordinate")
    a = args.get("angle_rad")
    if a is not None and abs(a) > MAX_PLAUSIBLE_RAD:
        return f"{abs(a):.1f} rad is more than four turns; confirm it"
    if d is not None and d < 0 and re.search(
            r"\b(?:forward|forwards|ahead)\b", clause, re.IGNORECASE):
        return ("the direction says forward but the distance is negative; "
                "say which you meant")
    return None


# ── Conditions on the robot's own measured pose (2026-09-24) ─────────
# "If your x is below 0.4 m, drive forward 0.7 m; otherwise stay put."
# Only x, y and heading, which the robot measures itself; only explicit
# comparisons with a number; both branches must parse cleanly. Anything else
# returns None and the sentence takes its usual path (the model). A condition
# on the world ("if the aisle is clear"), in the past ("if your x was ..."),
# negated, compound, or a heading with no unit is never evaluated here.
_COND_OPEN = re.compile(
    r"^\s*(?:if|provided(?:\s+that)?|in\s+case|assuming|should)\s+"
    r"(?P<abs>(?:the\s+)?absolute\s+value\s+of\s+)?"
    r"(?:your\s+|the\s+robot'?s\s+|its\s+|the\s+)?(?:current\s+|measured\s+)?"
    r"(?P<var>heading|yaw|orientation|bearing|x|y)\b"
    r"(?:[\s-]+(?:coordinate|position|value|reading|angle|axis))?"
    r"(?:\s+(?:is|be|reads|sits)\b|'s\b)?"
    r"(?:\s+(?:currently|already|now|right\s+now|still|somewhere))*\s*",
    re.IGNORECASE)
# "drive forward 0.8 m, but only if your x is below 0.5 m"
_COND_POSTFIX = re.compile(
    r"^(?P<act>.+?)\s*,?\s*(?:but\s+)?only\s+if\s+", re.IGNORECASE)
# "more than 20 degrees off zero": a size, not a signed value.
_OFF_ZERO = (r"(?:\s+(?P<off>off|away\s+from|from)\s+"
             r"(?:0|zero|straight\s+ahead|centre|center)\b)?")
_COND_NUM = r"(?P<{n}>[-+]?\d+(?:\.\d+)?)\s*(?P<{u}>" + _LEN_UNIT[:-2] + "|" + _ANG_UNIT[:-2] + r")?\b"
_COND_CMP = [
    ("between", re.compile(
        r"between\s+" + _COND_NUM.format(n="a", u="ua") + r"\s+and\s+"
        + _COND_NUM.format(n="b", u="ub"), re.IGNORECASE)),
    ("within", re.compile(
        r"within\s+" + _COND_NUM.format(n="a", u="ua") + r"\s+of\s+(?:zero\b|straight\s+ahead\b|"
        + _COND_NUM.format(n="b", u="ub") + r")", re.IGNORECASE)),
    ("le", re.compile(
        r"(?:<=|at\s+most|no\s+more\s+than|not\s+more\s+than|not\s+above|not\s+over)\s*"
        + _COND_NUM.format(n="a", u="ua"), re.IGNORECASE)),
    ("ge", re.compile(
        r"(?:>=|at\s+least|no\s+less\s+than|not\s+less\s+than)\s*"
        + _COND_NUM.format(n="a", u="ua"), re.IGNORECASE)),
    ("lt", re.compile(
        r"(?:<|below|under|less\s+than|lower\s+than|smaller\s+than)\s*"
        + _COND_NUM.format(n="a", u="ua") + _OFF_ZERO, re.IGNORECASE)),
    ("gt", re.compile(
        r"(?:>|above|over|greater\s+than|more\s+than|higher\s+than|larger\s+than|"
        r"bigger\s+than|exceeds|exceeding|beyond)\s*"
        + _COND_NUM.format(n="a", u="ua")
        + r"(?:\s+to\s+the\s+(?P<side>left|right)\b)?" + _OFF_ZERO, re.IGNORECASE)),
]
_COND_THEN = re.compile(r"\s*(?:,|;|:)?\s*(?:then\b)?\s*", re.IGNORECASE)
_COND_ELSE = re.compile(
    r"(?:[,;.]\s*|\s+)(?:otherwise|else|if\s+not|if\s+it\s+is\s+not|if\s+it'?s\s+not|"
    r"if\s+it\s+isn'?t|if\s+that\s+is\s+not\s+the\s+case|if\s+that'?s\s+not\s+the\s+case|"
    r"in\s+any\s+other\s+case|in\s+all\s+other\s+cases|in\s+every\s+other\s+case|"
    r"if\s+that'?s\s+false|if\s+that\s+is\s+false|if\s+it'?s\s+false|if\s+false)\b[,:]?\s*",
    re.IGNORECASE)
_NOOP = re.compile(
    r"(?:(?:stay|remain|hold|keep|stand)(?:\s+(?:still|put|where\s+you\s+are|in\s+place|"
    r"your\s+position|position|stationary))?|do\s+nothing|nothing|don'?t\s+move|"
    r"do\s+not\s+move|leave\s+it|no\s+action|skip\s+it|stop|halt|ignore\s+it)",
    re.IGNORECASE)


def _cond_value(var: str, value: str, unit: Optional[str]) -> float:
    """A threshold in metres (x, y) or radians (heading); ValueError if unusable."""
    u = (unit or "").lower()
    if var in ("x", "y"):
        if u and re.fullmatch(_ANG_UNIT[:-2], u, re.IGNORECASE):
            raise ValueError("an angle unit on a position")
        return _metres(value, u or None)
    if not u or not re.fullmatch(_ANG_UNIT[:-2], u, re.IGNORECASE):
        raise ValueError("a heading threshold needs an angle unit")
    return _radians(value, u)


def _read_condition(s: str, start: int = 0) -> Optional[Tuple[Dict[str, Any], int]]:
    """Parse "if <var> <comparison>" at `start` -> (condition, end index), or None."""
    head = _COND_OPEN.match(s, start)
    if head is None:
        return None
    var = head.group("var").lower()
    var = "yaw" if var in ("heading", "yaw", "orientation", "bearing") else var
    for op, rx in _COND_CMP:
        m = rx.match(s, head.end())
        if m:
            break
    else:
        return None
    g = m.groupdict()
    try:
        cond: Dict[str, Any] = {"var": var, "op": op,
                                "abs": bool(head.group("abs")) or bool(g.get("off")),
                                "a": _cond_value(var, g["a"], g.get("ua"))}
        if op == "between":
            cond["b"] = _cond_value(var, g["b"], g.get("ub") or g.get("ua"))
        elif op == "within":
            cond["b"] = (_cond_value(var, g["b"], g.get("ub") or g.get("ua"))
                         if g.get("b") else 0.0)
    except ValueError:
        return None
    if g.get("side") == "right":
        cond["negate"] = True        # "more than 30 degrees to the right"
    return cond, m.end()


def _state_conditional(raw: str, surface: str) -> Optional[Interpretation]:
    def branch(text: str) -> Optional[List[Frame]]:
        t = text.strip(" ,;.:!-")
        if not t:
            return None
        if _NOOP.fullmatch(_core(t) or ""):
            return []
        sub = interpret(t, surface)
        if sub.intent == COMMAND and sub.frames and not sub.residue \
                and sub.confidence >= 0.9:
            return sub.frames
        return None

    def build(cond: Dict[str, Any], then_text: str, else_text: Optional[str]
              ) -> Optional[Interpretation]:
        then_frames = branch(then_text)
        else_frames = branch(else_text) if else_text is not None else []
        if then_frames is None or else_frames is None:
            return None
        return Interpretation(CONDITIONAL, confidence=0.9, condition=cond,
                              then_frames=then_frames, else_frames=else_frames,
                              reason=f"a condition on the robot's measured {cond['var']}")

    # Postfix: "drive forward 0.8 m, but only if your x is below 0.5 m[;
    # otherwise ...]". The order comes first; what follows the comparison
    # may only be an otherwise-branch.
    post = _COND_POSTFIX.match(raw)
    if post is not None:
        probe = "if " + raw[post.end():]
        read = _read_condition(probe)
        if read is None:
            return None
        cond, end = read
        tail = probe[end:]
        if not tail.strip(" .!"):
            return build(cond, post.group("act"), None)
        split = _COND_ELSE.match(tail)
        if split is None:
            return None              # more condition than we can read
        return build(cond, post.group("act"), tail[split.end():])

    read = _read_condition(raw)
    if read is None:
        return None
    cond, end = read
    rest = raw[_COND_THEN.match(raw, end).end():]
    if re.match(r"(?:and|or|but)\b", rest, re.IGNORECASE):
        return None                  # a compound condition
    split = _COND_ELSE.search(rest)
    if split:
        return build(cond, rest[:split.start()], rest[split.end():])
    return build(cond, rest, None)


def evaluate_condition(cond: Dict[str, Any], pose: Sequence[float]) -> bool:
    """True when `cond` holds for pose (x, y, yaw); yaw in radians."""
    value = {"x": pose[0], "y": pose[1], "yaw": pose[2]}[cond["var"]]
    if cond.get("negate"):
        value = -value
    if cond.get("abs"):
        value = abs(value)
    a, op = cond["a"], cond["op"]
    if op == "lt":
        return value < a
    if op == "le":
        return value <= a
    if op == "gt":
        return value > a
    if op == "ge":
        return value >= a
    if op == "between":
        lo, hi = sorted((a, cond["b"]))
        return lo <= value <= hi
    if op == "within":
        return abs(value - cond.get("b", 0.0)) <= a
    raise ValueError(f"unknown comparison {op!r}")


SURFACES = (MOBILE, ARM, QUADRUPED, DRONE, ANY)


class UnknownSurface(ValueError):
    """`surface` was not one of SURFACES. See interpret()'s docstring."""


def interpret(text: str, surface: str = MOBILE) -> Interpretation:
    """Parse one operator utterance. Never raises ON THE TEXT; declines instead.

    ⚠️ IT DOES RAISE ON A BAD `surface`, AND THAT IS DELIBERATE.

    This argument used to accept anything. A rule only fires on a surface
    it applies to, so an unrecognised string matches no rule, and every
    utterance falls through to conversation / confidence 0.3 / no frames.
    That is indistinguishable from a parser that simply cannot understand
    the sentence.

    It cost a published result. `interpbench` was run with
    surface="MOBILE" -- uppercase -- and reported the deterministic parser
    at 68.3% with 19 missed commands, against the model's 91.7%, and
    concluded the model was the better interpreter by 23 points. With the
    case corrected the parser scores 90.0% with ONE missed command and the
    gap is 1.7 points. The measurement was of a typo.

    Declining silently is the right behaviour for a sentence nobody can
    parse and the wrong behaviour for a caller who named a robot class
    that does not exist -- one is a fact about the world, the other is a
    bug, and returning the same value for both is how the bug survived.
    Generalizing across four surfaces multiplies the exposure by four, so
    the trap is closed before the fan-out rather than after it.
    """
    if surface not in SURFACES:
        raise UnknownSurface(
            f"surface={surface!r} is not one of {list(SURFACES)}. "
            f"An unknown surface matches no rule and would degrade every "
            f"parse to conversation/no-frames, which reads as a parser "
            f"that understood nothing. Did you mean {str(surface).lower()!r}?")
    result = _interpret(text, surface)
    result.text = text or ""
    return result


def _interpret(text: str, surface: str = MOBILE) -> Interpretation:
    """The parse itself. Wrapped so every return path carries its text."""
    raw = _normalise(text)
    if not raw:
        return Interpretation(EMPTY, reason="nothing was said")

    low = raw.lower()

    # -- Guard 1: machine text is not an utterance -------------------
    if low.startswith(("[", "tool-output:")) or "```" in low:
        return Interpretation(CONVERSATION, reason="platform-injected text, "
                              "not an operator utterance", confidence=1.0)

    # -- Guard 2: challenges to the robot's own account --------------
    # Checked before everything, because these sentences are FULL of motion
    # verbs ("you never stopped") that mean the opposite of a command.
    if _SOCIAL.search(low):
        return Interpretation(CONVERSATION, confidence=0.9,
                              reason="a challenge to the robot's own account; "
                                     "only a model may answer it")

    # -- Guard 3: something to remember ------------------------------
    if _MEMORY_CUE.search(low):
        return Interpretation(MEMORY, reason="states a fact or policy to keep",
                              residue=raw, confidence=0.8)

    # -- A condition on the robot's own measured pose ------------------
    # Before the question guard: "Should your y be above 0.6 m, back up
    # 0.4 m" opens with a question word and is a condition. Returns None for
    # anything it cannot read exactly, and the sentence carries on below.
    if surface in (MOBILE, ANY):
        conditional = _state_conditional(raw, surface)
        if conditional is not None:
            return conditional

    # -- Guard 4: question form --------------------------------------
    # Three things have to be told apart, and the shipped ladder told apart
    # none of them:
    #   "can you stop"                  polite IMPERATIVE -> command
    #   "can you tell me where you are" polite QUESTION   -> query
    #   "when you've finished, pause"   SUBORDINATE clause -> not a question,
    #                                   even though it opens with a wh-word
    polite = bool(_POLITE_IMPERATIVE.match(raw))
    depoliced = _POLITE_IMPERATIVE.sub("", raw).strip() if polite else raw
    ends_q = raw.rstrip().endswith("?")
    # "do not take any new pickups" opens with `do`, which the question head
    # matches. A prohibition is never a question, and misreading one turns a
    # standing restriction into a status report.
    subordinate_head = bool(_SUBORDINATOR.match(raw)) or bool(_PROHIBITION.match(raw))

    def _as_query() -> Interpretation:
        kind = _query_kind(low)
        return Interpretation(
            QUERY, reason=f"question about {kind}"
            + (" (aggregate)" if _AGGREGATE.search(low) else ""),
            residue=raw, confidence=0.85,
            frames=[Frame("get_robot_state", {"kind": kind}, (0, len(raw)),
                          raw, "query")])

    if not subordinate_head:
        # An explicit hypothetical marker beats the polite-imperative rule:
        # "could you drive forward 2 metres, IN PRINCIPLE?" asks whether the
        # robot can, and the parser drove 2 m answering it.
        if _HYPOTHETICAL.search(low):
            return _as_query()
        # "tell me / show me ..." is an imperative in form and a question in
        # effect; answering it with a motion would be the ladder's mistake.
        if _TELL_ME.match(depoliced):
            return _as_query()
        if _QUESTION_HEAD.match(raw) and not polite:
            return _as_query()

    # -- Guard 5: prohibitions are standing constraints --------------
    if _PROHIBITION.search(low):
        return Interpretation(CONSTRAINT, reason="a standing restriction, not "
                              "a one-off action", residue=raw, confidence=0.8,
                              trigger=raw)

    # -- Guard 5b: an order that negates itself ----------------------
    # "drive forward 1 metre without moving" is not a command with a
    # caveat, it is incoherent -- and the parser drove the metre. Note the
    # negated verbs are motion verbs only, so "drive to the bay without
    # stopping" is left alone.
    if _SELF_NEGATING.search(low):
        return Interpretation(
            AMBIGUOUS, confidence=0.6, residue=raw,
            reason="that asks me to move and not move at once; say which")

    # -- Guard 6: a delay is a schedule, not an order for now --------
    delay = _DELAY.search(raw)
    if delay:
        return Interpretation(DEFERRED, trigger=delay.group(0),
                              reason=f"scheduled for {delay.group(0)}",
                              residue=raw, confidence=0.8)

    # -- Guard 7: a destination only the past can resolve ------------
    if _PAST_PLACE.search(raw):
        return Interpretation(
            AMBIGUOUS, confidence=0.7, residue=raw,
            reason="I would have to remember where that was; give me a "
                   "coordinate, or tell me to reset to home")

    # -- Guard 8: a retraction cancels what preceded it --------------
    # "drive forward 2 metres - no wait, make it 1 metre" drove 2 m: the
    # head won and the correction was residue. Running both would have
    # driven 3. The TAIL governs.
    # The FIRST marker, not the last: "make it" is itself a retraction
    # marker, so taking the last one ate the head of "drive 2 m - no wait,
    # make it 1 m" and left nothing to revise. A second retraction inside
    # the tail is handled by the recursive call below.
    # "... turn right 40 degrees instead. Finally, drive ..." -- an `instead`
    # that ENDS its sentence only flags a replacement; it starts nothing.
    raw_r = re.sub(r"\s*\b(?:instead|rather)\b(?=\s*[.;!])", "", raw,
                   flags=re.IGNORECASE)
    retraction = _RETRACTION.search(raw_r)
    if retraction is not None:
        head = raw_r[:retraction.start()].strip(" ,.-")
        tail = raw_r[retraction.end():].strip(" ,.-")
        # ⚠️ A correction inside a MULTI-STEP order is not applied here.
        # "Turn left 100 degrees. Then turn right 100 degrees - scratch
        # that, turn right 40 degrees. Finally, drive forward 0.4 m" kept
        # only the tail and drove 0.4 m: which earlier step a marker
        # retracts is not something to guess (found 2026-09-24, on the
        # already-spent v3 set, compound_v09). The model gets it.
        if head and len(interpret(head, surface).frames) >= 2:
            return Interpretation(
                AMBIGUOUS, confidence=0.5, residue=raw,
                reason="a correction inside a multi-step order")
        if not tail:
            # A trailing marker ("...3 metres INSTEAD") only flags that this
            # replaces something earlier; the head is still the order.
            return (interpret(head, surface) if head else
                    Interpretation(EMPTY, reason="nothing was said"))
        if _CANCEL_TAIL.search(tail):
            return Interpretation(
                CONVERSATION, confidence=0.85, residue=raw,
                reason="retracted, and the replacement cancels the order")
        sub = interpret(tail, surface)
        if sub.intent == COMMAND and sub.frames:
            return Interpretation(COMMAND, frames=sub.frames,
                                  confidence=min(sub.confidence, 0.85),
                                  reason=f"retracted; {sub.reason}",
                                  residue=sub.residue)
        # The tail may be only a new quantity -- "make it 1 metre" -- in
        # which case it re-fills the head's slot rather than replacing it.
        q = _REQUANTIFY.search(tail)
        base = interpret(head, surface) if head else None
        # ⚠️ The correction must be ONLY a new quantity. "turn right 60, then
        # drive 0.9 m -- actually change that drive to 0.5 m, then turn left
        # 60 degrees" re-filled the drive and DROPPED the final turn, so the
        # robot finished 60 degrees off (held-out v3, compound_v07, 2026-09-23).
        # Another step, number or direction after the quantity is the model's.
        after_q = _core(tail[:q.start()] + " " + tail[q.end():]) if q else ""
        if q and (re.search(r"\d", after_q) or _UNREAD_MEANING.search(
                re.sub(r"\b(?:" + _LEFT_WORDS + "|" + _RIGHT_WORDS + "|"
                       + _FWD_WORDS + "|" + _BWD_WORDS + r")\b", " ",
                       after_q, count=1, flags=re.IGNORECASE))
                  or re.search(r"\b(?:then|and then|after that|next|also)\b",
                               after_q, re.IGNORECASE)):
            return Interpretation(
                AMBIGUOUS, confidence=0.5, residue=raw,
                reason="a correction that also adds steps; I could not apply it "
                       "safely")
        if q and base is not None and base.intent == COMMAND and base.frames:
            unit = q.group(2)
            frame = base.frames[-1]
            # ⚠️ A direction in the correction wins over the head's. "turn
            # right 90 degrees, no, make it left 90 degrees" turned RIGHT:
            # the new magnitude kept the old sign.
            try:
                if re.fullmatch(_LEN_UNIT, unit, re.IGNORECASE) and \
                        "distance" in frame.args:
                    old = frame.args["distance"] or 1.0
                    sign = _correction_sign(tail, _FWD_WORDS, _BWD_WORDS)
                    frame.args["distance"] = math.copysign(
                        _metres(q.group(1), unit), sign or old)
                elif re.fullmatch(_ANG_UNIT, unit, re.IGNORECASE) and \
                        "angle_rad" in frame.args:
                    old = frame.args["angle_rad"] or 1.0
                    sign = _correction_sign(tail, _LEFT_WORDS, _RIGHT_WORDS)
                    frame.args["angle_rad"] = math.copysign(
                        _radians(q.group(1), unit), sign or old)
                else:
                    raise ValueError("a correction in the wrong unit")
            except ValueError:
                return Interpretation(
                    AMBIGUOUS, confidence=0.5, residue=raw,
                    reason="a correction I could not apply to the order")
            return Interpretation(COMMAND, frames=base.frames, confidence=0.8,
                                  reason=f"retracted; {base.reason} (revised)")
        return Interpretation(
            AMBIGUOUS, confidence=0.5, residue=raw,
            reason="something was retracted and I could not tell what "
                   "replaces it")

    # -- "turn right, 30 degrees": the comma is not a new step ------------
    # The clause split turned it into "turn right" (the 90-degree default)
    # plus an unreadable "30 degrees". Only a direction immediately followed
    # by a number is joined, so "turn right, then drive 2 m" is untouched.
    depoliced = _LEADING_DIR_COMMA.sub(r"\1 ", _COMMA_QUANTITY.sub(r"\1 ", depoliced or raw))
    for join in _JOIN_ONE_STEP:
        depoliced = join.sub(r"\1 ", depoliced)

    # -- Repetition ------------------------------------------------------
    rep = _repetition(depoliced or raw)
    if rep is not None:
        count, body, whole = rep
        sub = interpret(body, surface)
        if sub.intent == COMMAND and sub.frames and not sub.residue \
                and sub.confidence >= 0.9:
            if not whole and len(sub.frames) > 1:
                return Interpretation(
                    AMBIGUOUS, confidence=0.5, residue=raw,
                    reason="I could not tell whether the count applies to the "
                           "whole sequence or only the last step")
            if not 2 <= count <= _MAX_REPEAT or \
                    count * len(sub.frames) > _MAX_REPEATED_FRAMES:
                return Interpretation(
                    AMBIGUOUS, confidence=0.5, residue=raw,
                    reason=f"needs confirmation: {count} repetitions of "
                           f"{len(sub.frames)} step(s) is more than one order")
            frames = [Frame(f.tool, dict(f.args), f.span, f.source, f.rule)
                      for _ in range(count) for f in sub.frames]
            return Interpretation(COMMAND, frames=frames, confidence=0.9,
                                  reason=f"{count} x ({sub.reason})")
        # Otherwise parse it as if there were no count: a condition or a
        # constraint keeps its own handling, and a count left in a motion
        # clause is refused there as unread.

    # -- Parse the clauses -------------------------------------------
    frames: List[Frame] = []
    problems: List[str] = []
    residues: List[str] = []
    trigger: Optional[str] = None
    deferred = False
    hold = False

    for clause in _clauses(depoliced or raw):
        if not _core(clause):
            # ", please?", ", thanks", ", if you would": politeness split off
            # by a comma is not a clause the parser failed to read -- and it
            # must be skipped BEFORE the subordinator check, or "if you
            # would" reads as a condition on the whole order.
            continue
        sub = _SUBORDINATOR.search(clause)
        if sub:
            # Everything from the subordinator on is a CONDITION. The
            # imperative inside it ("...until I tell you to continue") is
            # emphatically not an order to execute now.
            head = clause[:sub.start()].strip()
            cond = clause[sub.start():].strip()
            if _HOLD_TRIGGER.search(cond):
                # Not a trigger to schedule: an indefinite hold to apply NOW.
                hold = True
            else:
                trigger = cond
                deferred = True
            if not head:
                continue
            clause = head

        frame, why, residue = _parse_clause(clause, surface)
        if frame is not None:
            frames.append(frame)
        else:
            problems.append(why)
            if residue:
                residues.append(residue)

    # "stop what you're doing and wait" is two clauses saying one thing.
    # Emitting stop() twice is harmless but it misreports what was understood.
    #
    # ⚠️ ADJACENT ONLY. The first version deduped globally, which silently
    # dropped the second leg of "drive forward 1 m, turn left 90, then drive
    # forward 1 m" -- the two drives are identical frames but a genuine
    # sequence, and the robot finished a metre short. Caught by commandbench
    # comp-3, which judges the pose rather than the reply.
    deduped: List[Frame] = []
    for f in frames:
        if deduped:
            prev = deduped[-1]
            if prev.tool == f.tool and prev.args == f.args:
                continue
        deduped.append(f)
    frames = deduped

    residue_text = "; ".join(r for r in residues if r)

    # A drive with no distance, and nothing else said: ask how far. Only when
    # the question IS the whole utterance -- no other frame (so "go ahead
    # and stop" is never answered with a question instead of a stop), no
    # number anywhere (so "drive forward, 2 metres" is not asked what it
    # already said) and no condition or hold.
    asks = [p for p in problems if p.startswith(_ASK_DISTANCE)]
    if asks and len(asks) == len(problems) and not frames and not deferred \
            and not hold and not re.search(r"\d", raw):
        question = asks[0][len(_ASK_DISTANCE):]
        return Interpretation(AMBIGUOUS, reason=question, residue=raw,
                              confidence=0.9, ask=question)

    if frames and deferred:
        # A release condition does not override the start condition:
        # "after this delivery, stop until I tell you" must finish the job.
        return Interpretation(DEFERRED, frames=frames, trigger=trigger,
                              reason="an action with a trigger", confidence=0.8,
                              residue=residue_text)

    if hold:
        # The whole point of recognising this: act now AND disable the 60 s
        # auto-resume, which is the behaviour that used to walk a "paused"
        # tug straight back into a forbidden cell.
        frames.append(Frame("hold", {"release": "operator"},
                            (0, len(raw)), raw, "hold"))
        return Interpretation(COMMAND, frames=frames, confidence=0.9,
                              reason="act now, then hold until the operator "
                                     "releases (no auto-resume)",
                              residue=residue_text)

    if frames:
        # Confidence falls when part of the utterance went unexplained: the
        # caller can require a higher bar before acting without confirming.
        # ⚠️ ANY unexplained clause puts it below the 0.8 acting floor, on
        # purpose and by a margin. This was `0.95 - 0.15 * n`, and one
        # residue came to 0.7999999999999999 -- so "drive forward 0.7 m, then
        # reverse 0.7 m to return to your start", parsed as the forward leg
        # ALONE, was declined only by floating-point rounding. A clause the
        # parser could not read may carry a step, a count or a direction.
        conf = 0.95 if not residues else max(0.5, 0.7 - 0.1 * (len(residues) - 1))
        return Interpretation(COMMAND, frames=frames, confidence=conf,
                              reason="; ".join(f.rule for f in frames),
                              residue=residue_text)
    if deferred and trigger:
        return Interpretation(DEFERRED, trigger=trigger, residue=residue_text,
                              reason="a trigger with no action this parser "
                                     "could fill", confidence=0.4)

    # Understood the verb but not the object -> worth asking about.
    ask_worthy = [p for p in problems
                  if "needs a referent" in p or "needs confirmation" in p]
    if ask_worthy:
        return Interpretation(AMBIGUOUS, reason=ask_worthy[0],
                              residue=residue_text or raw, confidence=0.6)

    # A question mark is evidence even when the body parsed as nothing: a
    # polite question ("can you tell me what you are doing now?") reaches
    # here only because the politeness guard let it through to be parsed.
    if ends_q:
        return _as_query()

    return Interpretation(CONVERSATION, reason="; ".join(problems) or
                          "no rule matched", residue=raw, confidence=0.3)
