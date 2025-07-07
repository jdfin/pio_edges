@echo off
setlocal
set pio_src="edges.pio"
..\data\packages\rp2040\tools\pqt-pioasm\4.0.1-8ec9d6f\pioasm.exe %pio_src% > %pio_src%.h
\Users\johnf\OneDrive\Documents\Programs\dos2unix\dos2unix.exe -q %pio_src%.h
