import cv2
import cv2.aruco as aruco
import numpy as np

def generate_charuco_board(squares_x, squares_y, square_length):
    marker_length = square_length/1.35

    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)

    image_size = (squares_x * 100, squares_y * 100) 
    try:
        board = aruco.CharucoBoard((squares_x, squares_y), square_length, marker_length, dictionary)
        board_image = board.generateImage(image_size, marginSize=10, borderBits=1)
    except:
        board = aruco.CharucoBoard_create(squares_x, squares_y, square_length, marker_length, dictionary)
        board_image = board.draw((image_size[0], image_size[1]))


    return board_image, board

def detect_charuco(frame, board, dictionary):
    if len(frame.shape) > 2:
        frame= cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    marker_corners, marker_ids, _ = aruco.detectMarkers(frame, dictionary)

    if marker_ids is None or len(marker_ids) == 0:
        return None, None, marker_corners, marker_ids

    retval, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
        marker_corners,
        marker_ids,
        frame,
        board
    )

    if retval is None or retval < 4:
        return None, None, marker_corners, marker_ids

    return charuco_corners, charuco_ids, marker_corners, marker_ids

def generate_heatmap(size, all_corners):
    print("GENERATING HEATMAP FROM",len(all_corners),"POINTS")
    image = np.full((size[0], size[1], 1), 255, dtype=np.uint8)
    for corners in all_corners:
        for corner in corners:
            corner = corner[0]
            image[int(corner[1])][int(corner[0])] = 0
    
    dist_map = cv2.distanceTransform(image, cv2.DIST_C, 3)
    return cv2.normalize(dist_map, dist_map, 0, 255.0, cv2.NORM_MINMAX)