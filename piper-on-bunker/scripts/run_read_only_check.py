from piper_on_bunker.cli import main

if __name__ == "__main__":
    import sys
    if len(sys.argv) == 1:
        sys.argv = [sys.argv[0], "read-only", "--config", "piper-on-bunker/config/piper_laptop_dry_run.yaml"]
    main()
