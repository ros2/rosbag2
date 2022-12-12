

class BagCommandExtension:
    def get_compression_modes(self):
        return ['none', 'chunk']

    def get_compression_formats(self):
        return ['lz4']

    def get_preset_profiles(self):
        return ['none', 'fastwrite', 'zstd_fast', 'zstd_small']
