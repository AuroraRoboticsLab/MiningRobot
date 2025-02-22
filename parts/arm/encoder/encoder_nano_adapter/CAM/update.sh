#!/bin/sh

cmd="$HOME/cnc/software_pcb/pcb2gcode/pcb2gcode/pcb2gcode --metric --metricoutput --zchange=5 --zsafe=1.5  --nog64 --nog81 --nom6   --mirror-yaxis=1 "


$cmd  --zwork=-0.3  --offset=0.6 --zdrill=-3.0 --drill-feed 400 --drill-speed 10000  --drill *-PTH.drl

$cmd --zwork=-0.4  --offset=0.6 --mill-feed 400 --mill-speed 10000 --cutter-diameter=0.3   --back *"-B_Cu.gbr" 

$cmd --zcut=-1.8 --cutter-diameter=1.0  --cut-feed=200 --cut-infeed=100 --cut-speed=10000  --cut-side=back --outline *"-Edge_Cuts.gbr" 

