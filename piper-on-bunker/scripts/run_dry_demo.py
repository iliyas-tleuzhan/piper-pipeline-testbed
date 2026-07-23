from piper_on_bunker.cli import main

if __name__ == "__main__":
    import sys
    sys.argv = [sys.argv[0], "live-dry-run", "--config", "piper-on-bunker/config/piper_laptop_dry_run.yaml"]
    main()
