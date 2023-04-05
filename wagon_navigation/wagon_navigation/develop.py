import matplotlib.pyplot as plt
import numpy as np
from scipy import interpolate


def fritz_Carlson(points, resolution):
                
    # Calculate the distances between each pair of points
    distances = np.sqrt(np.sum(np.diff(points, axis=0)**2, axis=1))

    # Calculate the cumulative distance along the curve
    cumulative_distances = np.cumsum(distances)
    cumulative_distances = np.insert(cumulative_distances, 0, 0) # Add initial distance of 0

    # Calculate tangents at each point using the Fritsch-Carlson method
    d = np.diff(points, axis=0) / np.diff(cumulative_distances)[:, None]
    u = np.zeros_like(points)
    u[1:-1] = np.where(d[1:] * d[:-1] <= 0, 0, (np.abs(d[1:]) * d[:-1] + np.abs(d[:-1]) * d[1:]) / (np.abs(d[1:]) + np.abs(d[:-1])))
    u[0] = d[0]
    u[-1] = d[-1]
    tangents = u / np.sqrt(np.sum(u**2, axis=1))[:, None]

    # Create a cubic Hermite spline interpolation of the points
    interp = interpolate.CubicHermiteSpline(cumulative_distances, points, tangents)
 
    # Generate points along the curve at the specified resolution
    s_vals = np.array([])
    for idx, dist in enumerate(cumulative_distances[:-1], ):
        num_points = int(np.ceil((cumulative_distances[idx + 1] - dist)/resolution))
        s_val = np.linspace(dist, cumulative_distances[idx + 1], num_points)
        s_vals = np.append(s_vals, s_val[1:])

    # Generate spline points along the curve
    interp_points = interp(s_vals)

    return interp_points

def Hermite(points, resolution):
                
    # Concatenate the points to form a 3x3 array
    # points = np.array([p1, p2, p3])

    # Calculate the distances between each pair of points
    distances = np.sqrt(np.sum(np.diff(points, axis=0)**2, axis=1))

    # Calculate the cumulative distance along the curve
    cumulative_distances = np.cumsum(distances)
    cumulative_distances = np.insert(cumulative_distances, 0, 0) # Add initial distance of 0

    # Create a cubic spline interpolation of the points
    # interp = interpolate.CubicSpline(cumulative_distances, points, bc_type='not-a-knot')

    # Calculate tangents at each point using the central difference method
    tangents = np.zeros_like(points)
    tangents[0] = (points[1] - points[0]) / (cumulative_distances[1] - cumulative_distances[0])
    for i in range(1, len(points)-1):
        tangents[i] = (points[i+1] - points[i-1]) / (cumulative_distances[i+1] - cumulative_distances[i-1])
    tangents[-1] = (points[-1] - points[-2]) / (cumulative_distances[-1] - cumulative_distances[-2])

    # Create a cubic Hermite spline interpolation of the points
    interp = interpolate.CubicHermiteSpline(cumulative_distances, points, tangents)
 
    # Generate points along the curve at the specified resolution
    s_vals = np.array([])
    for idx, dist in enumerate(cumulative_distances[:-1], ):
        num_points = int(np.ceil((cumulative_distances[idx + 1] - dist)/resolution))
        # print(num_points)
        s_val = np.linspace(dist, cumulative_distances[idx + 1], num_points)
        s_vals = np.append(s_vals, s_val[1:])

    # Generate 10 points along the curve
    interp_points = interp(s_vals)
    

    
    return interp_points

def gen_spline(points, resolution):
            

    # Calculate the distances between each pair of points
    distances = np.sqrt(np.sum(np.diff(points, axis=0)**2, axis=1))

    # Calculate the cumulative distance along the curve
    cumulative_distances = np.cumsum(distances)
    cumulative_distances = np.insert(cumulative_distances, 0, 0) # Add initial distance of 0

    # Create a cubic spline interpolation of the points
    interp = interpolate.CubicSpline(cumulative_distances, points, bc_type='not-a-knot')

    # Generate points along the curve at the specified resolution
    s_vals = np.array([])
    for idx, dist in enumerate(cumulative_distances[:-1], ):
        num_points = int(np.ceil((cumulative_distances[idx + 1] - dist)/resolution))
        # print(num_points)
        s_val = np.linspace(dist, cumulative_distances[idx + 1], num_points)
        s_vals = np.append(s_vals, s_val[1:])

    # Generate 10 points along the curve
    interp_points = interp(s_vals)

    return interp_points


def main():

    waypoints = np.array([[8.50931,3.333,-0.206],[5.20931,3.333,-0.206],[10.20931,2.333,-0.206], [14.20931,6.333,-0.206], [20.20931,6.333,-0.206], [26.20931,6.333,-0.206], [28.20931,2.533,-0.206], [30.20931,2.533,-0.206], [32.20931,6.533,-0.206]])
    interp_points = gen_spline(waypoints, 0.5)
    hermite_points = Hermite(waypoints, 0.5)
    #fritz_Carlson_points = fritz_Carlson(waypoints, 0.5)
        # Plot the original points as crosses and the spline points as dots
    fig, ax = plt.subplots()
    ax.plot(interp_points[:, 0], interp_points[:, 1], 'o', label='Spline Points')
    ax.plot(hermite_points[:, 0], hermite_points[:, 1], 'o', label='Hermite Points')
    #ax.plot(fritz_Carlson_points[:, 0], fritz_Carlson_points[:, 1], 'o', label='Fritz Carlson Points')
    ax.plot(waypoints[:, 0], waypoints[:, 1], 'x', label='Original Points')
    # rotate the plot with 180 degrees and flip it
    ax.set_xlim(ax.get_xlim()[::-1])
    ax.set_ylim(ax.get_ylim()[::-1])
    ax.legend()
    ax.set_aspect('equal') # Set the axes to be equal
    plt.show()


main()