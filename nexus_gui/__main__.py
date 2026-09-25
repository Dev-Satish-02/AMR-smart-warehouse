import argparse

import uvicorn


def main():
    parser = argparse.ArgumentParser(description="NEXUS control room (live view + layout editor)")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    args = parser.parse_args()
    print(f"NEXUS control room -> http://{args.host}:{args.port}")
    uvicorn.run("nexus_gui.server:app", host=args.host, port=args.port, log_level="warning")


if __name__ == "__main__":
    main()
