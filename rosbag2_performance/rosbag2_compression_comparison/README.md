# Compression Size Comparison

This folder contains a script to convert a bag into a variety of different compressed formats, in order to compare the resulting file sizes.

To use this script, you need a bag (sqlite3 or MCAP format) containing a representative distribution of messages for your
use-case. Then use `run.sh` with your bag as an argument:

```
./run.sh <your bag path>
```

This should print a csv-formatted list of names + sizes in bytes for each compression technique.
