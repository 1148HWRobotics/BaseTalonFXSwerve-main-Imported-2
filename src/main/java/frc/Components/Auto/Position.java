package frc.Components.Auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.math.AngleMath;
import frc.lib.util.Vector2;

// Importing the Vector2 class, which represents a 2-dimensional vector for position.

// The Position class encapsulates the angle and 2D position data for an object, such as a robot on the field.
public class Position {
    public double angle; // The angle variable likely represents the orientation of the object in
                         // degrees.
    public Vector2 position; // The position variable is a 2D vector representing the location of the object.

    // Constructor for the Position class.
    // It initializes the angle and position of the object.
    public Position(double angle, Vector2 position) {
        this.angle = angle; // Sets the angle of the object.
        this.position = position; // Sets the 2D position of the object.
    }

    public Pose2d asPose2d() {
        return new Pose2d(new Translation2d(position.x, position.y), new Rotation2d(angle));
    }

    public Position difference(Position other) {
        return new Position(AngleMath.getDelta(other.angle, this.angle), this.position.minus(other.position));
    }

    public Position combine(Position other, double weight) {
        return new Position(AngleMath.blendAngles(other.angle, this.angle, weight),
                this.position.multiply(weight).add(other.position.multiply(1 - weight)));
    }

    public Position add(Position other) {
        return new Position(AngleMath.conformAngle(other.angle + this.angle), this.position.add(other.position));
    }

    public Position scale(double scaleFactor) {
        return new Position(AngleMath.conformAngle(this.angle * scaleFactor), this.position.multiply(scaleFactor));
    }

    public String toString() {
        return position.toString() + ", " + angle;
    }
}
