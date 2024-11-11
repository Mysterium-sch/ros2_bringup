import numpy as np
import cv2
from cv2 import aruco
from statemachine import StateMachine, State


ARUCO_DICT_NAME = aruco.DICT_ARUCO_ORIGINAL


def create_tag_detector(hist_size=20):
    """Create a aruco tag detector with debouncing."""
    aruco_dict = aruco.getPredefinedDictionary(ARUCO_DICT_NAME)
    params = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, params)

    history = []

    def detector_(img):
        nonlocal history

        _, tag_ids, _ = detector.detectMarkers(img)

        # If more or less than one tag is detected, tag detection is ambiguous.
        if tag_ids is None or len(tag_ids) != 1:
            tag_id = None
        else:
            tag_id = tag_ids[0][0]

        # Append current tag in front and truncate history to it's size.
        history = ([tag_id] + history)[:hist_size]

        # Count consecutive entries of the detected tag in the history.
        cnt = 0
        while cnt < len(history):
            if history[cnt] == tag_id:
                cnt += 1
            else:
                break

        percent = cnt / float(hist_size)
        return (tag_id, percent)

    def clearer():
        nonlocal history
        history = []

    return detector_, clearer



class TagStateMachine(StateMachine):
    """A state machine to handle tag detection behavior."""

    def __init__(self):
        self.current_tag_id = None
        super().__init__()

        self.allow_event_without_transition = True

    scanning = State(initial=True)
    new_tag_detected = State()
    tag_in_progress = State()
    detected_tag_used = State()

    update = (
        scanning.to(tag_in_progress, cond=["nonempty_tag"]) |
        tag_in_progress.to(new_tag_detected, cond=["progress_100", "nonempty_tag"]) |
        tag_in_progress.to(scanning, cond=["progress_50", "different_tag"]) |
        new_tag_detected.to(scanning, cond=["progress_50", "different_tag"]) |
        detected_tag_used.to(scanning, cond=["progress_50", "different_tag"]))

    consume_tag = (
        new_tag_detected.to(detected_tag_used))

    def progress_100(self, tag_id, tag_progress):
        return tag_progress == 1.0

    def progress_50(self, tag_id, tag_progress):
        return tag_progress >= 0.5

    def progress_20(self, tag_id, tag_progress):
        return tag_progress >= 0.2

    def nonempty_tag(self, tag_id, tag_progress):
        return tag_id is not None

    def different_tag(self, tag_id, tag_progress):
        return tag_id != self.current_tag_id

    @tag_in_progress.enter
    def enter_tag_in_progress(self, tag_id, tag_progress):
        self.current_tag_id = tag_id

    @new_tag_detected.enter
    def enter_new_tag_detected(self, tag_id, tag_progress):
        self.current_tag_id = tag_id

    @scanning.enter
    def enter_scanning(self):
        self.current_tag_id = None
