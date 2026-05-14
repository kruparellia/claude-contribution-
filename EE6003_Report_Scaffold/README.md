# EE6003 Robotic Arm — Scaffold (team write-up version)

This is the **scaffold copy** of the report. Same structure, same diagrams,
same bibliography as the main draft — but section bodies are replaced with
guidance bullets and short placeholder paragraphs for the team to write into.

Use this version when the team wants to write the report in their own voice
while still being steered towards top-band marks.

## How to use

1. Zip this `EE6003_Report_Scaffold/` folder.
2. Overleaf → **New Project → Upload Project → Zip File**.
3. Compiler: **pdfLaTeX**. Main file: `main.tex`.

## What's in each section file

Every section file uses three custom helpers:

- `\guide{...}` — a yellow-tinted "guidance box" listing what to write.
- `\placeholder{...}` — a short placeholder paragraph (replace with prose).
- `\corehit{...}` / `\advhit{...}` — keep these; they evidence the brief's
  60% / advanced-feature labelling. Edit the wording if your final content
  shifts.

When you're done writing a section, **delete the `\guide{}` block** for
that section so it doesn't appear in the final PDF. The `\placeholder{}`
text should be overwritten with your real prose. The `\corehit/\advhit`
lines stay.

## What's already filled in

- Figures: power architecture, HC-05 divider, arm wiring, controller wiring.
- Bibliography: 13 IEEE-style references seeded for the literature review.
- Brief-aligned section structure mapped 1:1 onto the marking rubric.
- Brief's **core vs advanced** labelling embedded in every technical section.
- Word count target reminders embedded as comments at the top of each file.

## Word count target (from the brief)

The brief asks for **4500 words** (excluding code, tables, figures,
references, appendices). Anything more than 10% over (≥4950 words) is
capped at 40%. Each section file lists a suggested target — they sum to
~4400 leaving headroom.

## Marking rubric mapping

| Rubric row                         | Marks | Section file               |
|------------------------------------|-------|----------------------------|
| Abstract / Intro / Lit Review      | 10    | 01_abstract, 02_introduction, 03_literature_review |
| Embedded Design & Architecture     | 10    | 04_system_design           |
| Hardware Development & Sensors     | 20    | 05_hardware                |
| Software & Bluetooth Comm.         | 15    | 06_software                |
| Robotic Arm Design & 3D Printing   | 10    | 07_arm_design              |
| Testing & Performance Evaluation   | 20    | 08_testing                 |
| Teamwork & Professional Feedback   | 10    | 09_teamwork                |
| Report Writing & Documentation     | 5     | (whole report; structure)  |

## Top-marks reminder

The single most important sentence in the brief: **core requirements alone
cap you at 60%**. Every technical section in this scaffold has an explicit
`\advhit{}` slot — fill it. If a section has no advanced claim, you're
leaving marks on the table.
