import rosbag2_py._rosbag2_py as rosbag2_py


def get_rosbag_options(path, serialization_format='cdr'):
    storage_options = rosbag2_py.StorageOptions()
    storage_options.uri = path
    storage_options.storage_id = 'sqlite3'

    converter_options = rosbag2_py.ConverterOptions()
    converter_options.input_serialization_format = serialization_format
    converter_options.output_serialization_format = serialization_format

    return storage_options, converter_options
