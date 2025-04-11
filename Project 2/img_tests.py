
def detectTest():
    detector = Detect()
    detector.set_lower_mask(114, .6, .2)
    detector.set_upper_mask(154, 1, 1)

    r1.ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)

    while True:
        try:
            frame = r1.ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        except Empty:
            time.sleep(0.001)
            continue

        masked = detector.mask_image_pls(frame)
        edges = detector.edges(masked)
        center = detector.center(edges, 15)
        edges_bgr = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

        if center:
            center_x, center_y = center
            vertical_lines = detector.sides(edges, center_x)

            # Draw on color image
            edges_bgr = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
            for x1, y1, x2, y2 in vertical_lines:
                cv2.line(edges_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Green
            cv2.circle(edges_bgr, center, 10, (0, 0, 255), -1)  # Red dot

            # cv2.imshow('Camera', edges_bgr)
            print(f"Distance: {detector.distance(detector.line_length(vertical_lines))}")
            # print(detector.line_length(vertical_lines))

        # Display the captured frame
        cv2.imshow('Camera', edges_bgr)

        # Press 'q' to exit the loop
        if cv2.waitKey(1) == ord('q'):
            break

def areaTest():
    detector = Detect()
    r1.ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)

    while True:
        try:
            frame = r1.ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        except Empty:
            time.sleep(0.001)
            continue

        mask = detector.detect_object(frame, detector.PAPER_ORANGE)
        edges = detector.edges(mask)

        brick = detector.isolate_brick(edges)
        # distance = detector.distance_area(brick)

        #print(f"Distance: {distance}")

        # Display the captured frame
        cv2.imshow('Camera', brick)

        # Press 'q' to exit the loop
        if cv2.waitKey(1) == ord('q'):
            break
