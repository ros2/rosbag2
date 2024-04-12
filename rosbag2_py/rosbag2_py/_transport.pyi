from typing import Any, ClassVar, List

import datetime
import rosbag2_py._storage

class PlayOptions:
    clock_publish_frequency: float
    clock_publish_on_topic_publish: bool
    clock_topics: List[str]
    delay: float
    disable_keyboard_controls: bool
    disable_loan_message: bool
    exclude_regex_to_filter: str
    exclude_service_events_to_filter: List[str]
    exclude_topics_to_filter: List[str]
    loop: bool
    node_prefix: str
    playback_duration: float
    playback_until_timestamp: int
    publish_service_requests: bool
    rate: float
    read_ahead_queue_size: int
    regex_to_filter: str
    service_requests_source: Any
    services_to_filter: List[str]
    start_offset: float
    start_paused: bool
    topic_qos_profile_overrides: dict
    topic_remapping_options: List[str]
    topics_to_filter: List[str]
    wait_acked_timeout: int
    def __init__(self) -> None: ...

class Player:
    def __init__(self) -> None: ...
    def burst(self, storage_options: rosbag2_py._storage.StorageOptions, play_options: PlayOptions, num_messages: int) -> None: ...
    def cancel(self, *args, **kwargs) -> Any: ...
    def play(self, storage_options: rosbag2_py._storage.StorageOptions, play_options: PlayOptions) -> None: ...

class RecordOptions:
    all_services: bool
    all_topics: bool
    compression_format: str
    compression_mode: str
    compression_queue_size: int
    compression_threads: int
    exclude_regex: str
    exclude_service_events: List[str]
    exclude_topic_types: List[str]
    exclude_topics: List[str]
    ignore_leaf_topics: bool
    include_hidden_topics: bool
    include_unpublished_topics: bool
    is_discovery_disabled: bool
    node_prefix: str
    regex: str
    rmw_serialization_format: str
    services: List[str]
    start_paused: bool
    topic_polling_interval: datetime.timedelta
    topic_qos_profile_overrides: dict
    topic_types: List[str]
    topics: List[str]
    use_sim_time: bool
    def __init__(self) -> None: ...

class Recorder:
    def __init__(self) -> None: ...
    def cancel(self, *args, **kwargs) -> Any: ...
    def record(self, storage_options: rosbag2_py._storage.StorageOptions, record_options: RecordOptions, node_name: str = ...) -> None: ...

class ServiceRequestsSource:
    __doc__: ClassVar[str] = ...  # read-only
    __members__: ClassVar[dict] = ...  # read-only
    CLIENT_INTROSPECTION: ClassVar[ServiceRequestsSource] = ...
    SERVICE_INTROSPECTION: ClassVar[ServiceRequestsSource] = ...
    __entries: ClassVar[dict] = ...
    def __init__(self, value: int) -> None: ...
    def __eq__(self, other: object) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: object) -> bool: ...
    def __setstate__(self, state: int) -> None: ...
    @property
    def name(self) -> str: ...
    @property
    def value(self) -> int: ...

def bag_rewrite(arg0: List[rosbag2_py._storage.StorageOptions], arg1: str) -> None: ...
