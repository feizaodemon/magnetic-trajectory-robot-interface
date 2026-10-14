"""Colors and fonts shared by the dashboard widgets. No Tk import."""

FONT_FAMILY = "TkDefaultFont"

APP_BG = "#F1F5F9"
SURFACE = "#FFFFFF"
SURFACE_MUTED = "#F8FAFC"
BORDER = "#E2E8F0"
BORDER_STRONG = "#CBD5E1"

HEADER_BG = "#0F172A"
HEADER_TEXT = "#F8FAFC"
HEADER_MUTED = "#94A3B8"
HEADER_CHIP = "#1E293B"
HEADER_CHIP_ACTIVE = "#334155"

TEXT = "#0F172A"
TEXT_SECONDARY = "#475569"
TEXT_MUTED = "#94A3B8"

ACCENT = "#4F46E5"
STROKE = ACCENT
STROKE_HALO = "#C7D2FE"

# Tone name -> (strong color for text, bars and stripes; soft tint for fills).
TONES = {
    "neutral": ("#64748B", "#F1F5F9"),
    "info": ("#2563EB", "#DBEAFE"),
    "accent": (ACCENT, "#EEF2FF"),
    "success": ("#16A34A", "#DCFCE7"),
    "warning": ("#D97706", "#FEF3C7"),
    "danger": ("#DC2626", "#FEE2E2"),
}


def tone_colors(tone):
    """``(strong, soft)`` colors of a tone; unknown tones are neutral."""
    return TONES.get(tone, TONES["neutral"])


def font(size, bold=False):
    return (FONT_FAMILY, int(size), "bold") if bold else (FONT_FAMILY, int(size))
