# EE6003 Robotic Arm — Overleaf LaTeX Project

This folder is a **standalone Overleaf-ready** LaTeX project for the EE6003
Embedded Systems group report (Advanced Industrial Robotic Arm using RF
Wireless Communication for Industrial Applications).

## How to use it

1. Zip this entire `EE6003_Report/` folder.
2. On Overleaf: **New Project → Upload Project → Zip File**.
3. Set the compiler to **pdfLaTeX** (Menu → Settings → Compiler).
4. The main file is `main.tex`.

## Structure

```
EE6003_Report/
├── main.tex                  ← entry point, includes everything
├── references.bib            ← IEEE-style bibliography
├── sections/                 ← one .tex per section, easy to edit/swap
│   ├── 01_abstract.tex
│   ├── 02_introduction.tex
│   ├── 03_literature_review.tex
│   ├── 04_system_design.tex
│   ├── 05_hardware.tex
│   ├── 06_software.tex
│   ├── 07_arm_design.tex
│   ├── 08_testing.tex
│   ├── 09_teamwork.tex
│   └── 10_conclusion.tex
└── figures/                  ← PDF/SVG diagrams + photo placeholders
    ├── 01_power_architecture.pdf
    ├── 02_hc05_voltage_divider.pdf
    ├── 03_arm_wiring.pdf
    └── 04_controller_wiring.pdf
```

## Notes for k

- Word-count target this draft: ~2500–3000 of the 4500-word brief budget.
  Plenty of headroom for results numbers, the assembly write-up, and the
  individual contribution paragraphs you'll add later.
- Every section that has *core* and *advanced* content uses an explicit
  `\textbf{Core requirement addressed:}` / `\textbf{Advanced feature claim:}`
  line so the moderator can find the 60%-cap evidence at a glance.
- Photo placeholders are clearly marked with TODO comments — search the
  `.tex` files for `% TODO PHOTO` to find every spot that wants a build pic.
- The "Hardware Implementation" section is intentionally light on the
  physical-build detail since you said someone else will fill that in.
- Bibliography is pre-seeded with the references already written into your
  literature-review draft. Add more as you go and they'll be picked up
  automatically.
