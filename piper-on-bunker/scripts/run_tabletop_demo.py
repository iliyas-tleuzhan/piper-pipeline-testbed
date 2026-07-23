from piper_on_bunker.cli import main

if __name__ == "__main__":
    import sys
    args = list(sys.argv[1:])
    require_physical = "--require-physical-enable" in args
    config = "piper-on-bunker/config/piper_laptop_hardware.yaml" if require_physical else "piper-on-bunker/config/piper_laptop_dry_run.yaml"
    command = "hardware-demo" if require_physical else "live-dry-run"
    sys.argv = [sys.argv[0], command, "--config", config]
    main()
