# A very rough script to run batches of experiments. Missing checks and command line parametrization

trap ctrlc SIGINT
ctrlc_sent=0

function ctrlc()
{
  let ctrlc_sent=1
}

test_dir=/tmp/rosbag2_test/$(date +%Y%m%d_%H%M%S)
mkdir -p ${test_dir}
echo "${test_dir} created"
db_path=${test_dir}/bag
rm -fr ${db_path}
script_dir=$(dirname $(readlink -f "$0"))

storage_config_dir=${script_dir}/storage_config
config_optimized_path=${storage_config_dir}/storage_optimized.yaml
config_default_path=${storage_config_dir}/storage_default.yaml

for storage_config_file_path in ${config_optimized_path} ${config_default_path}
do
  config_file_name=$(basename $storage_config_file_path)
  summary_file=${test_dir}/results_${config_file_name}.csv
  for cache in 0 1000000 100000000 500000000
  do
    for sz in 10000 100000 1000000 5000000
    do
      for inst in 1 10 100 1000
      do
        freq=100; #Hz
        let total=$inst*$sz*$freq
        if [[ $total -lt 100000000 || $total -gt 500000000 ]]; then
          #echo "skipping the case of ${inst} instances and size ${sz}"
          continue
        fi
        echo "processing case of ${inst} instances and size ${sz} with cache ${cache} and config ${config_file_name}"
        outdir=${test_dir}/size${sz}_inst${inst}_cache${cache}
        mkdir -p ${outdir}
        for try in 1 2 3
        do
          outfile=${outdir}/${try}.log
          echo "Results will be written to file: ${outfile}"
          ros2 run rosbag2_performance_writer_benchmarking writer_benchmark --ros-args \
            -p frequency:=${freq} \
            -p size:=${sz} \
            -p instances:=${inst} \
            -p max_cache_size:=${cache} \
            -p db_folder:=${db_path} \
            -p storage_config_file:=${storage_config_file_path} \
            -p results_file:=${summary_file} \
            --ros-args -r __node:=rosbag2_performance_writer_benchmarking_node_batch \
            2> ${outfile}
          sleep 2 # making sure to flush both application and disk caches and have a fresh start
          cp ${db_path}/metadata.yaml ${outdir}/${try}_metadata.yaml # preserve metadata from test
          rm -fr ${db_path} # but remove large database file so we won't run out of disk space
          if [[ $ctrlc_sent -eq 1 ]]; then
            echo -e "\e[31mQuitting prematurely due to Ctrl-C - some results aren't saved and some won't be reliable\e[0m]"
            exit
          fi
        done
      done
    done
  done
done
