package frc.robot;

public class OutputTriangle {
    private OutputPoint point1, point2, point3;
    private OutputTriangle neighbor1, neighbor2, neighbor3;

    public OutputTriangle() {
    }

    public OutputTriangle(OutputPoint p1, OutputPoint p2, OutputPoint p3) {
        this.point1 = p1;
        this.point2 = p2;
        this.point3 = p3;
    }

    public OutputTriangle(OutputPoint p1, OutputPoint p2, OutputPoint p3, OutputTriangle n1, OutputTriangle n2,
            OutputTriangle n3) {
        this.point1 = p1;
        this.point2 = p2;
        this.point3 = p3;
        this.neighbor1 = n1;
        this.neighbor1 = n2;
        this.neighbor3 = n3;
    }

    public OutputTriangle(double[][] array, int[] intervals) {
        point1 = new OutputPoint(array, intervals[0]);
        point2 = new OutputPoint(array, intervals[1]);
        point3 = new OutputPoint(array, intervals[2]);
    }

    public OutputPoint getPoint1() {
        return point1;
    }

    public OutputPoint getPoint2() {
        return point2;
    }

    public OutputPoint getPoint3() {
        return point3;
    }

    public OutputTriangle getNeighbor1() {
        return neighbor1;
    }

    public OutputTriangle getNeighbor2() {
        return neighbor2;
    }

    public OutputTriangle getNeighbor3() {
        return neighbor3;
    }

    public void setPoint1(OutputPoint point1) {
        this.point1 = point1;
    }

    public void setPoint2(OutputPoint point2) {
        this.point2 = point2;
    }

    public void setPoint3(OutputPoint point3) {
        this.point3 = point3;
    }

    public void setNeighbor1(OutputTriangle neighbor1) {
        this.neighbor1 = neighbor1;
    }

    public void setNeighbor2(OutputTriangle neighbor2) {
        this.neighbor2 = neighbor2;
    }

    public void setNeighbor3(OutputTriangle neighbor3) {
        this.neighbor3 = neighbor3;
    }

    public OutputPoint getPoint(int x) {
        switch (x) {
            case 1 -> {
                return point1;
            }
            case 2 -> {
                return point2;
            }
            case 3 -> {
                return point3;
            }
            default -> {
                System.out.println("Invalid call for triangle point");
                return new OutputPoint();
            }
        }
    }

    public void setPoint(int x, OutputPoint tempPoint) {
        switch (x) {
            case 1 -> {
                point1 = tempPoint;
            }
            case 2 -> {
                point2 = tempPoint;
            }
            case 3 -> {
                point3 = tempPoint;
            }
            default -> {
                System.out.println("Invalid set command for triangle point");
            }
        }
    }

    public OutputTriangle getNeighbor(int x) {
        switch (x) {
            case 1 -> {
                return neighbor1;
            }
            case 2 -> {
                return neighbor2;
            }
            case 3 -> {
                return neighbor3;
            }
            default -> {
                System.out.println("Invalid call for triangle neighbor");
                return new OutputTriangle();
            }
        }
    }

    public void setNeighbor(int x, OutputTriangle tempTri) {
        switch (x) {
            case 1 -> {
                neighbor1 = tempTri;
            }
            case 2 -> {
                neighbor2 = tempTri;
            }
            case 3 -> {
                neighbor3 = tempTri;
            }
            default -> {
                System.out.println("Invalid set command for triangle point");
            }
        }
    }

    public OutputPoint[] getAllPoints() {
        return new OutputPoint[] { getPoint1(), getPoint2(), getPoint3() };
    }

    public OutputTriangle[] getAllNeighbors() {
        return new OutputTriangle[] { getNeighbor1(), getNeighbor2(), getNeighbor3() };
    }

    public void setAllPoints(OutputPoint[] newPoints) {
        setPoint1(newPoints[0]);
        setPoint2(newPoints[1]);
        setPoint3(newPoints[2]);
    }

    public void setAllNeighbors(OutputTriangle[] newTriangles) {
        setNeighbor1(newTriangles[0]);
        setNeighbor2(newTriangles[1]);
        setNeighbor3(newTriangles[2]);
    }
}
