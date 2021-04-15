cd `dirname $0`

for path in $(ls ./)
do
  if [ -d $path ] ; then
    cd $path
    source setup.sh;
    cd ..
  fi
done