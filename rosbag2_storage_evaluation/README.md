# ROS 2.0 Rosbag Evaluation

## Benchmarks

This folder currently contains benchmarks which measure the write speed and disk usage of SQLite files.

The single table schema
```
  sqlite::create_table(db, "MESSAGES", {
    "TIMESTAMP INTEGER NOT NULL",
    "TOPIC TEXT NOT NULL",
    "DATA BLOB NOT NULL"
  });
```
is compared with a foreign key schema storing topics in a separate table
```
  sqlite::create_table(db, "TOPICS", {
    "ID INTEGER PRIMARY KEY",
    "TOPIC TEXT NOT NULL"
  });

  sqlite::create_table(db, "MESSAGES", {
    "TIMESTAMP INTEGER NOT NULL",
    "TOPIC_ID INTEGER NOT NULL",
    "DATA BLOB NOT NULL"
  }, {sqlite::ForeignKeyDef{"TOPIC_ID", "TOPICS", "ID"}});
```

It should be **easy to add additional bag file formats**, e.g. for writing directly to disk or writing the RosBag 2.0 format.

### Build from command line

The project is using cmake. The script `./build.sh` can be used to build it.

This will generate benchmark binaries in `./build/bin/`.

### Run

Individual benchmarks in `./build/bin` can be run by hand. To run the complete suite the script `./run_all_benchmarks.sh` can be used.

Each benchmark will generate a CSV file in `./build/bin` containing the measured data for further plotting with the Jupyter Notebook.

## Jupyter Notebook

It is used for data analysis and visualization.

### Setup

Prerequisites: Python 3.5+, [pip](https://pip.pypa.io/en/stable/), [virtualenv](https://virtualenv.pypa.io/en/stable/)

```
virtualenv -p python3 venv
. venv/bin/activate
pip install -r requirements.txt
```

### Usage

```
. venv/bin/activate
jupyter notebook data_analysis_and_visualization.ipynb
```

A browser window should open. Click `Cell -> Run All`.

## Extending the benchmarks

### Read Tests

To measure retrieval time of the first message, extend the `MessageWriter` interface like so
```
  virtual MessageStream::Ptr selectAll() = 0;
  virtual MessageStream::Ptr selectTopic(std::string const & topic) = 0;
  virtual MessageStream::Ptr selectFromTo(
    Message::Timestamp const & fromInclusive, Message::Timestamp const & toExclusive) = 0;
```
where `MessageStream` is a lazy data structure
```
  virtual bool has_next() const = 0;
  virtual MessagePtr next() = 0;
```
implemented with a streaming SQLite `SELECT` statement.

The desired timings can then be measured with the `Profiler`.
