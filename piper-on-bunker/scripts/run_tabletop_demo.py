from piper_on_bunker.cli import main

if __name__ == "__main__":
    import sys
    config = "piper-on-bunker/config/piper_laptop_hardware.yaml" if "--require-physical-enable" in sys.argv else "piper-on-bunker/config/piper_laptop_dry_run.yaml"
    sys.argv = [sys.argv[0], "dry-demo", "--config", config]
    main()
