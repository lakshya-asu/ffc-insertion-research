"""Fail closed until a macro-specific evaluated checkpoint is installed."""


def main():
    raise RuntimeError(
        "Macro inference requires a separately trained and evaluated checkpoint; none selected yet"
    )
