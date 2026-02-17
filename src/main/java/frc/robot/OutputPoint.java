package frc.robot;

public class OutputPoint {
    private double x, y;
    private double[] data;

    public OutputPoint() {
    }

    public OutputPoint(double x, double y, double[] data) {
        this.x = x;
        this.y = y;
        this.data = data;
    }

    public OutputPoint(double[][] array, int interval) {
        data = new double[array[0].length - 3];
        for (int i = 0; i < array.length; i++) {
            if (array[i][0] == interval) {
                x = array[i][1];
                y = array[i][2];
                for (int j = 3; j < array[i].length; j++) {
                    data[j - 3] = array[i][j];
                }
            }
        }
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double[] getData() {
        return data;
    }

    public double getOneData(int pos) {
        return data[pos];
    }

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }

    public void setData(double[] data) {
        this.data = data;
    }

    public void setOneData(int pos, double oneData) {
        data[pos] = oneData;
    }
}
