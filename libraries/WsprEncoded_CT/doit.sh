
for f in *.h ; do
    sed -i 's!WSPR_WSPR!WSPR!g' $f
done
