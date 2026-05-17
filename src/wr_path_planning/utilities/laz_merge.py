import sys
import json
import numpy as np
import pdal

def main(argv=None):
    n = len(argv)
    if n < 3:
        raise Exception("Usage: <laz1> <laz2> ... <lazN> <output>")

    input_files = argv[1:(n - 1)]
    output_file = argv[n - 1]

    pipeline = [
        *input_files,
        {
            "type": "filters.merge"
        },
        {
            "type": "writers.las",
            "filename": output_file,
            "compression": "laszip"
        }
    ]

    pipeline_json = json.dumps(pipeline)
    operator = pdal.Pipeline(pipeline_json)
    operator.execute()


if __name__ == "__main__":
    argv = sys.argv
    main(argv)