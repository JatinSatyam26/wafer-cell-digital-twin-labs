# Wafer-Cell Digital Twin: Short-Course Labs

Six hands-on labs on building and using a live, read-only **digital twin** of a two-robot wafer-handling cell (Dobot M1 Pro SCARA, myCobot Pro 600, conveyor, laser station, inspection camera, Micro850 PLC) at Arizona State University, under the guidance of Dr. Sangram Redkar.

This repository is formatted for the robotics-lab website. It follows the same layout as the existing labs: one `_labs/<lab>/index.md` per lab, with Jekyll front matter, and all files under `assets/Projects/<project>/`.

## Contents

| Path | What |
|---|---|
| `_labs/lab-00-digital-twin/` | Start here: course overview, the cell, safety, network setup, reference appendices, acknowledgments |
| `_labs/lab-01-digital-twin/` | Lab 1: Reading the Cell: The Read-Only Bridge |
| `_labs/lab-02-digital-twin/` | Lab 2: Building the Live Digital Twin |
| `_labs/lab-03-digital-twin/` | Lab 3: Wafer Handling with Contact Physics |
| `_labs/lab-04-digital-twin/` | Lab 4: Vision Trained in Simulation |
| `_labs/lab-05-digital-twin/` | Lab 5: Fault Detection from Motion |
| `_labs/lab-06-digital-twin/` | Lab 6: Making the Real Cell Faster with the Twin |
| `assets/Projects/Wafer-Cell-Digital-Twin/figures/` | Every figure the lab pages use (web-sized) |
| `assets/Projects/Wafer-Cell-Digital-Twin/pdf/` | The printable manuals: the full manual and one PDF per lab |
| `assets/Projects/Wafer-Cell-Digital-Twin/photos/` | Photos of the cell at the bench |
| `assets/Projects/Wafer-Cell-Digital-Twin/README.md` | Project overview |

The course data the labs use (`course_materials.zip`: the cell's reference files, PLC logs and the Lab 3 slip-test recordings, about 74 MB) is attached to the [v1.0 release](https://github.com/JatinSatyam26/wafer-cell-digital-twin-labs/releases/tag/v1.0) rather than stored in the repository.

The twin's code is in its own public repository: **https://github.com/JatinSatyam26/wafer-cell-digital-twin** (the labs cite commit `75990ed`).

## Adding these labs to the website

1. Copy the seven folders in `_labs/` into the site's `_labs/` folder.
2. Copy `assets/Projects/Wafer-Cell-Digital-Twin/` into the site's `assets/Projects/`.
3. The pages use `layout: lab` and `lab_series: digital-twin`, and each has a permalink of the form `/labs/lab-0N-digital-twin/`. Asset links are site-absolute (`/assets/Projects/Wafer-Cell-Digital-Twin/...`).

## Acknowledgments

This work was carried out at Arizona State University under the guidance of Dr. Sangram Redkar. The wafer-handling cell that the twin observes, its PLC program, and its robot bridge scripts were built and written by Alonso Preciado Martinez.

Author of the digital twin and these labs: Jatin Satyam.
