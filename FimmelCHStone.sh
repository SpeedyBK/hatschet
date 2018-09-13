#!/bin/bash


start_scheduler()
{
useScheduler=$1
output=$2
benchmarkPart=$3
benchmarkPartCount=$4
shift; shift; shift; shift;

i=3
while [ $i -le $benchmarkPartCount ]
do

graphFile="benchmarks/ChStone/${benchmarkPart}/graph${i}.graphml"
ressourceFile="benchmarks/ChStone/${benchmarkPart}/graph${i}_RM.xml"

./hatschet --scheduler=$useScheduler --resource=$ressourceFile --graph=$graphFile --dot=test >> $output

echo "§§§Fin" >> $output

i=`expr $i + 1`

done

}


useScheduler1=MOOVAC
useScheduler2=RATIONALIIFIMMEL
output=output.txt

> $output

start_scheduler $useScheduler1 $output adpcm 10
start_scheduler $useScheduler2 $output adpcm 10
