
for f in *.h ; do
    # sed -i 's!WSPR_WSPR!WSPR!g' $f
    sed -i 's!_H_$!_H!g' $f
done
