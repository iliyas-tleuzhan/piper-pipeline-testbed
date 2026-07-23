from piper_on_bunker.cli import main

if __name__ == "__main__":
    import sys
    sys.argv = [sys.argv[0], "mock-demo", "--config", "piper-on-bunker/config/development_mock.yaml"]
    main()
