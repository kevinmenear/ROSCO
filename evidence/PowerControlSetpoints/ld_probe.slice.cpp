std::string field(const std::string& text, int w) {
    if (static_cast<int>(text.size()) > w) {
        return std::string(static_cast<size_t>(w), '*');
    }
    return std::string(static_cast<size_t>(w) - text.size(), ' ') + text;
}

// gfortran renders a non-finite value as a WORD, not as digits: "NaN" with no
// sign, "Infinity" carrying it. Copied from debug.cpp, minus the narrow-field
// "Inf" arm -- the field here is 26 and "Infinity" is 8, so that arm has no
// reachable input at this one descriptor and an unreachable arm is worse than
// an absent one. Returns false when the value is finite.
bool nonfinite_text(double v, std::string& out) {
    if (std::isnan(v)) { out = "NaN"; return true; }
    if (std::isinf(v)) {
        out = std::signbit(v) ? "-Infinity" : "Infinity";
        return true;
    }
    return false;
}

// One list-directed record for a REAL(8), without the terminator.
std::string list_directed_real(double v) {
    std::string text;
    if (nonfinite_text(v, text)) {
        return field(text, 26);
    }

    char tmp[512];

    // The decimal exponent is READ OFF the rounded conversion rather than
    // computed with log10: 17 significant digits identify a double uniquely, so
    // this is the exponent the printed digits actually have, and there is no
    // boundary where a log10 and a rounding disagree.
    std::snprintf(tmp, sizeof tmp, "%.16E", v);
    std::string s(tmp);
    const size_t epos = s.find('E');
    const int decexp = std::atoi(s.c_str() + epos + 1);

    // 0.1 <= |v| < 1e17, and v == 0 lands here because "%.16E" of a zero
    // reports an exponent of 0 -- which is the arm gfortran puts it in.
    if (decexp >= -1 && decexp <= 16) {
        // 17 significant digits: `decexp + 1` of them are before the point.
        const int d = 16 - decexp;
        std::snprintf(tmp, sizeof tmp, "%.*f", d, v);
        text = tmp;
        if (d == 0) {
            text += '.';
        }
        return field(text, 21) + std::string(5, ' ');
    }

    const std::string mant = s.substr(0, epos);
    const char esign = s[epos + 1];
    std::string edig = s.substr(epos + 2);
    while (edig.size() > 1 && edig[0] == '0') {
        edig.erase(0, 1);
    }
    if (edig.size() > 3) {
        return field(std::string(27, '*'), 26);
    }
    edig.insert(0, 3 - edig.size(), '0');
    return field(mant + "E" + esign + edig, 26);
}
