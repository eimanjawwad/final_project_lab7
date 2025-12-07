# karel_test.py

import karel
import time


def main():
    pupper = karel.KarelPupper()
    
    print("\n=== Testing aim_middle (neutral standing) ===")
    pupper.aim_middle()
    print("Holding middle pose for 3 seconds...")
    time.sleep(0.5)
    
    print("\n=== Testing aim_up ===")
    pupper.aim_up(percent=60.0)
    print("Holding up pose for 3 seconds...")
    time.sleep(2)

    print("\n=== Resuming walking mode ===")
    pupper.resume_walking()
    print("Done!")

if __name__ == '__main__':
    main()