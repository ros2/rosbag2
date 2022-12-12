from rosbag2_py import get_registered_compressors

class BagCommandExtension():
    def get_compression_modes(self):
        return ['none', 'message', 'file']

    def get_compression_formats(self):
        return list(get_registered_compressors())

    def get_preset_profiles(self):
        return ['none', 'resilient']
