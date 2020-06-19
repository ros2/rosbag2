
test_dir=/tmp/rosbag2_test
echo ${test_dir}
db_path=${test_dir}/bag
rm -fr ${db_path}
mkdir -p ${db_path}

freq=100; #Hz

for cache in 0 10 100 1000
do
  for sz in 1000 10000 100000 1000000
  do
    for inst in 1 10 100 1000
    do
      let total=$inst*$sz
      if [[ $total -ne 1000000 ]]; then
        #echo "skipping the case of ${inst} instances and size ${sz}"
        continue
      fi
      echo "processing case of ${inst} instances and size ${sz} with cache ${cache}"
      #if [[ ${inst} -eq 1000 && ${s} -eq 1000000 ]]; then
      #  echo "skipping the case of ${inst} instances and size ${s}"
      #  continue
      #fi
      for try in 1 2 3 4 5
      do
        outdir=${test_dir}/size${sz}_inst${inst}_cache${cache}
        mkdir -p ${outdir}
        outfile=${outdir}/${try}.log
        echo "Results will be written to file: ${outfile}"
        ros2 run writer_benchmarking writer_benchmark --ros-args -p frequency:=${freq} -p size:=${sz} -p instances:=${inst} -p max_cache_size:=${cache} -p db_folder:=${db_path} 2> ${outfile}
        rm -fr ${db_path}/bag*
      done
    done
  done
done
