from movementfix import Movement


def main():
    movement = Movement()
    try:
        movement.start()

        # movement.rov(time, angle/yaw, depth_cm, surge/vx, sway/vy)
        depth = 10  # dalam cm
        print("maju")
        movement.rov(100, 0, depth, 200, 0)
        # print("maju")
        # movement.rov(10, 0, depth, 0,  200)
        print("menjaga posisi")
        movement.rov(10, 0, depth, 0,  0)

        # movement.go_to_depth(60)  # turun ke 60 cm dan hold sampai stabil


    except KeyboardInterrupt:
        print("\nstop!")
    finally:
        movement.cleanup()


if __name__ == "__main__":
    main()