from piper_on_bunker.cli import main

if __name__ == "__main__":
    import sys
    sys.argv = [sys.argv[0], "replay-demo", "--config", "piper-on-bunker/config/tabletop_replay.yaml"]
    main()
