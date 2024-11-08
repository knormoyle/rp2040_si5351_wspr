#!/usr/bin/env bash
source /home/kevin/Downloads/spy/errorhandle
DIR1='/home/kevin/sf-hab_rp2040_picoballoon_tracker_pcb_gen1/firmware/LightAPRS-W-2.0/LightAPRS-W-2-pico-balloon'
DIR2='/home/kevin/rp2040_si5351_wspr/LightAPRS-W-2-pico-balloon'
FILE='LightAPRS-W-2-pico-balloon.ino'

echo "see how I updated to be able to git submodule update --remote by "
echo "with the right branch in /home/kevin/sf-hab_rp2040_picoballoon_tracker_pcb_gen1/firmware/LightAPRS-W-2.0"
echo "by modifying the root .gitmodules in sf-hab_rp2040_picoballon_tracker_pcb_gen1"

pushd /home/kevin/sf-hab_rp2040_picoballoon_tracker_pcb_gen1
# no..don't want this. not most recent?
# git submodule update
git submodule update --remote

# why do these have different # of lines?
# I did this here, to get this LightAPRS-W-2-pico-balloon.ino

# The ported version of the repository is added to this project as a git submodule, so once you clone this repository you need to do:
# git submodule init
# git submodule update


echo ""
echo "$DIR1/$FILE"
cd $DIR1
wc -l $FILE
# 1728 LightAPRS-W-2-pico-balloon.ino

echo ""

# I forked a repo and got the branch
echo ""
echo "$DIR2/$FILE"
cd $DIR2
wc -l $FILE
# 1745 LightAPRS-W-2-pico-balloon.ino


echo ""
echo "diff $DIR1/$FILE $DIR2/$FILE"
diff $DIR1/$FILE $DIR2/$FILE

popd

# port_to_ag6ns_rp2040_picoballoon_tracker

# 
# /home/kevin/sf-hab_rp2040_picoballoon_tracker_pcb_gen1/firmware/LightAPRS-W-2.0/LightAPRS-W-2-pico-balloon/LightAPRS-W-2-pico-balloon.ino
# 1745 LightAPRS-W-2-pico-balloon.ino
# 
# 
# /home/kevin/rp2040_si5351_wspr/LightAPRS-W-2-pico-balloon/LightAPRS-W-2-pico-balloon.ino
# 1745 LightAPRS-W-2-pico-balloon.ino
# 
# diff /home/kevin/sf-hab_rp2040_picoballoon_tracker_pcb_gen1/firmware/LightAPRS-W-2.0/LightAPRS-W-2-pico-balloon/LightAPRS-W-2-pico-balloon.ino /home/kevin/rp2040_si5351_wspr/LightAPRS-W-2-pico-balloon/LightAPRS-W-2-pico-balloon.ino
# 
