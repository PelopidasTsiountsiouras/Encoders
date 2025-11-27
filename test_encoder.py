#!/usr/bin/env python3

import time
import logging
from encoder_tracker import EncoderTracker   # <-- imports your class


def setup_logging(level=logging.INFO):
    logger = logging.getLogger("EncoderTest")
    logger.setLevel(level)
    logger.handlers.clear()

    fmt = logging.Formatter(
        "%(asctime)s - %(levelname)s - %(message)s",
        datefmt="%H:%M:%S"
    )

    ch = logging.StreamHandler()
    ch.setFormatter(fmt)
    logger.addHandler(ch)

    return logger


def main():

    # ----------------------------------------------------
    # EDIT THESE VALUES FOR YOUR SETUP
    # ----------------------------------------------------
    PIN_A = 17
    PIN_B = 27
    PIN_INDEX = 22          # or None if not using Z
    PPR = 48                # Your encoder's PPR
    WHEEL_DIAM_MM = 60      # in mm
    # ----------------------------------------------------

    logger = setup_logging(logging.INFO)

    logger.info("===============================================")
    logger.info("        QUADRATURE ENCODER TEST MONITOR")
    logger.info("===============================================")
    logger.info(f"A={PIN_A}, B={PIN_B}, Index={PIN_INDEX}")
    logger.info("Rotate the wheel forward/backwards slowly and fast.")
    logger.info("Check that rotations, distance, and direction are correct.")
    logger.info("CTRL+C to exit.\n")

    enc = EncoderTracker(
        pin_a=PIN_A,
        pin_b=PIN_B,
        ppr=PPR,
        wheel_diameter_mm=WHEEL_DIAM_MM,
        pin_index=PIN_INDEX,
        logger=logger
    )

    last_print = time.time()
    last_diag = time.time()

    try:
        while True:
            now = time.time()

            # Show measurements every 1 second
            if now - last_print >= 1:

                count = enc.get_count()
                rot = enc.get_rotations()
                dist = enc.get_distance_cm()
                speed = enc.get_speed_cmps()
                idx = enc.get_index_count()

                logger.info(
                    f"Count: {count:7d} | "
                    f"Rot: {rot:8.3f} | "
                    f"Dist: {dist:8.2f} cm | "
                    f"Speed: {speed:6.2f} cm/s | "
                    f"Index: {idx}"
                )

                last_print = now

            # Show diagnostics every 5 seconds
            if now - last_diag >= 5:
                diag = enc.get_diagnostics()
                logger.info("----------- DIAGNOSTICS -----------")
                for key, val in diag.items():
                    logger.info(f"{key:20s}: {val}")
                logger.info("-----------------------------------\n")
                last_diag = now

            time.sleep(0.05)

    except KeyboardInterrupt:

        logger.info("\n===============================================")
        logger.info("                FINAL RESULTS")
        logger.info("===============================================")
        logger.info(f"Final count        : {enc.get_count()}")
        logger.info(f"Final rotations    : {enc.get_rotations():.3f}")
        logger.info(f"Final distance (cm): {enc.get_distance_cm():.2f}")
        logger.info(f"Index pulses       : {enc.get_index_count()}")
        logger.info("===============================================")

    finally:
        enc.cleanup()
        logger.info("GPIO cleaned up. Bye!")


if __name__ == "__main__":
    main()
