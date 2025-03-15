import numpy as np

def de_casteljau_1d(points, t):
    """
    De Casteljau's algorithm for a single parameter t and given control points.
    :param points: List of control points (coordinates as floats).
    :param t: Parameter t (float in [0, 1]).
    :return: Resulting coordinate after applying the algorithm.
    """
    while len(points) > 1:
        points = [(1 - t) * points[i] + t * points[i + 1] for i in range(len(points) - 1)]
    return points[0]


def evaluate_surface(control_points, u0, v0):
    """
    Evaluates a point on the Bézier surface using iterated De Casteljau's algorithm.
    :param control_points: 3D list/array of control points with shape (n+1)x(m+1).
    :param u0: Parameter along the u direction (float in [0, 1]).
    :param v0: Parameter along the v direction (float in [0, 1]).
    :return: Coordinates of the point r(u0, v0).
    """
    # Apply De Casteljau along the u direction for each row (v is fixed)
    intermediate_points = []
    for row in control_points:
        intermediate_points.append(de_casteljau_1d(row, u0))

    # Apply De Casteljau again on intermediate points (along v direction)
    result = de_casteljau_1d(intermediate_points, v0)
    return result


def read_input_file(file_path):
    """
    Reads input data from a file.
    :param file_path: Path to the input file.
    :return: Degrees n and m, control points list, and parameters u0 and v0.
    """
    with open(file_path, 'r') as f:
        lines = [line.strip() for line in f if line.strip()]  # Remove empty lines
        n, m = map(int, lines[0].split())
        control_points = []
        index = 1
        for _ in range(n + 1):
            row = []
            for i in range(m + 1):
                if index < len(lines):
                    row.append(list(map(float, lines[index].split())))
                    index += 1
                else:
                    raise ValueError(
                        "Input file has incorrect format. Check the number of control points.")
            control_points.append(row)
        if index < len(lines):
            u0, v0 = map(float, lines[index].split())
        else:
            raise ValueError("Parameters u0 and v0 are missing in the input file.")
    return n, m, np.array(control_points), u0, v0


def main():
    # Path to input file
    file_path = 'input_bezier.txt'

    # Read data
    try:
        n, m, control_points, u0, v0 = read_input_file(file_path)
        print(f"Surface degrees: n={n}, m={m}")
        print(f"Parameters: u0={u0}, v0={v0}")

        # Compute resulting point
        result = evaluate_surface(control_points, u0, v0)
        print(f"Resulting point r(u0, v0): {result}")
    except ValueError as e:
        print(f"Error: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")


if __name__ == "__main__":
    main()