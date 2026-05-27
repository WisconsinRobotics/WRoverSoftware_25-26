import sys
import json
import pdal


def main(argv=None):
    n = len(argv)
    if n < 5:
        raise Exception("Usage: python3 merge.py voxel_size <laz1> <laz2> ... <lazN> <output>")
    
    voxel_size = float(argv[1])
    input_files = argv[2:n - 1]
    output_file = argv[n - 1]

    pipeline_dict = {
        "pipeline": [
            *input_files,
            {
                "type": "filters.merge"
            },
            {
                "type": "filters.voxeldownsize",
                "cell": voxel_size,
                "mode": "center"
            },
            {
                "type": "writers.las",
                "filename": output_file,
                "compression": "laszip"
            }
        ]
    }

    pipeline_json = json.dumps(pipeline_dict)
    pipeline = pdal.Pipeline(pipeline_json)
    pipeline.execute_streaming()

    print(f"Merged {len(input_files)} file(s) into {output_file}")


if __name__ == "__main__":
    main(sys.argv)