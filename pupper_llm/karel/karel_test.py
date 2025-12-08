# karel_test.py

import karel
import time


def main():
    pupper = karel.KarelPupper()
    
    # print("\n=== Testing aim_up ===")
    # tp = pupper.aim_up(percent=100.0)
    # print("Holding up pose for 3 seconds...")
    # time.sleep(2)

    # print("\n=== Testing aim_middle (neutral standing) ===")
    # pupper.aim_middle(target_pose=tp)
    # print("Holding middle pose for 3 seconds...")
    # time.sleep(2.0)

    # print("\n=== Resuming walking mode ===")
    # pupper.resume_walking()
    # print("Done!")

    pupper.press_trigger()
    time.sleep(1)
    print("done shooting")

if __name__ == '__main__':
    main()