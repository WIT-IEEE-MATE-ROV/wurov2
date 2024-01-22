import sys, glob, os, time
import threading
from threading import Thread
from multiprocessing import Queue, Process
from typing import List

import cv2
import numpy.typing as npt
from typing import Tuple



class Camera:
    """
    Initialize the `Camera` class with parameters for capturing frames as seprate daemon thread.

    Args:
        width (int): Desired width of the captured frames.
        height (int): Desired height of the captured frames.
        queue (Queue): A multi-producer, multi-consumer queue to store captured frames.
        source (str): The video source string, either a device index or file path.

    Raises:
        ValueError: If the requested resolution is not supported by the camera.
        RuntimeError: If the camera cannot be opened.

    Attributes:
        width (int): The actual width of the captured frames.
        height (int): The actual height of the captured frames.
        is_running (bool): Whether the camera thread is running.
        queue (Queue): The queue used to store captured frames.
        read_lock (threading.Lock): A lock to synchronize access to the frame buffer.
        capture (cv2.VideoCapture): The OpenCV video capture object.
        grabbed (bool): Whether the first frame was successfully grabbed.
        frames (numpy.ndarray): The latest captured frame, or None if no frame is available.
    """

    def __init__(self, width: int, height: int, source: str) -> None:
        self.width = width
        self.height = height
        self.source = source
        self.is_running = False
        # self.queue = q
        self.read_lock = threading.Lock()
        # Video capture APIs
        # https://docs.opencv.org/3.4/d4/d15/group__videoio__flags__base.html#gga023786be1ee68a9105bf2e48c700294dab6ac3effa04f41ed5470375c85a23504
        self.capture = cv2.VideoCapture(source, cv2.CAP_ANY)
        self.grabbed, self.frames = self.capture.read()
        try:
            self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        except ValueError:
            print(f"resloution not supported ({self.width, self.height})")


    def __exit__(self, exec_type, exc_value, traceback) -> None:
        self.cap.release()

    def start(self) -> object:
        """
        Starts the video capture in a separate thread.

        Raises:
            RuntimeError: If the capture is already started.

        Returns:
            self: The Camera object itself, allowing for method chaining.

        """
        if self.is_running:
            print(f"capture is already running on {self.source}")
            return None
        self.is_running = True
        self.thread = Thread(target=self._update, args=(), daemon=True)
        self.thread.start()
        if self.thread.is_alive():
            print(
                f"Camera capture started on {self.source} [THREAD:{self.thread.name}] [PID: {os.getpid()}]"
            )
        return self

    def _update(self) -> None:
        """
        Continuously captures frames from the camera in a background thread.

        Updates the `grabbed` and `frames` attributes of the Camera object with the latest captured frame.

        Employs a lock (`read_lock`) to ensure thread-safe access to these attributes.

        Raises:
            RuntimeError: If the camera capture fails.
        """
        while self.is_running:
            grabbed, frames = self.capture.read()
            with self.read_lock:
                self.grabbed = grabbed
                self.frames = frames

    def read(self) -> Tuple[bool, npt.NDArray]:
        """
        Reads the most recently captured frame from the camera.

        Returns:
            A tuple of (grabbed, frame):
            - grabbed (bool): True if a frame was successfully read, False otherwise.
            - frame (npt.NDArray): The captured frame as a NumPy array, or None if no frame is available.

        Raises:
            ValueError: If the camera is not running.
        """
        with self.read_lock:
            if self.frames is None:
                raise AttributeError("frame is None type")
            frame = self.frames.copy()
            grabbed = self.grabbed
        return grabbed, frame

    def stop(self):
        """
        Stops running thread and releases capture
        """
        self.is_running = False
        self.thread.join()
        self.capture.release()
        cv2.destroyAllWindows()

    def view(self, frames):
        """
        Displays the video capture.

        Exits on "q" key press or KeyboardInterrupt (Ctrl+C).

        """
        try:
            cv2.imshow(f" {self.thread.name} ", frames)
            if cv2.waitKey(1) == ord("q"):
                self.stop()
                sys.exit("exiting")
        except KeyboardInterrupt as ex:
            print("exit")
            self.stop()


    def save_frame(self, path: str):
        """
        When function is called current frame is witten to given path
        """
        if self.is_running:
            _, current_frame = self.read()
            cv2.imwrite(path, current_frame)
    

    @staticmethod
    def list_devices() -> List[str]:
        """
        Lists available video capture devices on the system.

        Returns:
            A list of strings containing the paths to accessible video devices (e.g., "/dev/video0", "/dev/video1").
        """
        devices = []
        video = glob.glob("/dev/video*")
        for vid in video:
            cap = cv2.VideoCapture(vid)
            if cap.grab():
                devices.append(vid)
            else:
                pass
        return devices



if __name__ == "__main__":
    frame_buff = Queue(maxsize=512)
    print(Camera.list_devices())
    ct = Camera(width=480, height=620, source="/dev/video0")
    ct.start()
  
    try:
        count = 0
        while True:
            _, frame = ct.read()
            time.sleep(0.5)
            ct.save_frame(f"./frame_{count}.png")
            count += 1 
            if count > 10:
                sys.exit()
            #ct.view(box_frame)
    except KeyboardInterrupt:
        print("stop")
        frame_buff.join()