package frc.robot;

import java.io.File;
import java.math.BigDecimal;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

// import org.poly2tri.triangulation.TriangulationPoint;
// import org.poly2tri.triangulation.point.TPoint;

import com.github.cliftonlabs.json_simple.JsonArray;
import com.github.cliftonlabs.json_simple.JsonObject;
import com.github.cliftonlabs.json_simple.Jsoner;

public class PointHandler {
    public PointHandler() {
    }

    private static double getArrayData(JsonArray array, int interval, String key) {
        try {
            JsonObject m_temp = (JsonObject) (array.get(interval));
            if (m_temp.containsKey(key)) {
                BigDecimal bD = (BigDecimal) (m_temp.get(key));
                return bD.doubleValue();
            } else {
                return -1;
            }

        } catch (Exception e) {
            return -1;
        }
    }

    private static int dataTypeNum(JsonArray array) {
        JsonObject m_temp = (JsonObject) array.get(0);
        JsonArray m_data = (JsonArray) (m_temp.get("data"));
        return m_data.size();
    }

    private static double[] getArrayList(JsonArray array, int interval) {
        JsonObject m_temp = (JsonObject) array.get(interval);
        JsonArray m_data = (JsonArray) (m_temp.get("data"));
        double[] dataList = new double[m_data.size()];
        for (int i = 0; i < m_data.size(); i++) {
            BigDecimal listBD = (BigDecimal) m_data.get(i);
            dataList[i] = listBD.doubleValue();
        }
        return dataList;
    }

    private static boolean isTriangulable(double[][] data) {
        double[] m_xList = new double[data.length];
        double[] m_yList = new double[data.length];

        for (int i = 0; i < data.length; i++) {
            m_xList[i] = data[i][1];
            m_yList[i] = data[i][2];
        }

        for (int i = 0; i < m_xList.length; i++) {
            for (int j = 0; j < m_xList.length; j++) {
                if (m_xList[i] == m_xList[j] && m_yList[i] == m_yList[j] && i != j) {
                    System.out.println("Repeating point: (" + m_xList[i] + ", " + m_yList[i] + ")");
                    return false;
                }
            }
        }

        return true;

        // LIST LOGIC TEST CODE:
        // System.out.print("\n[");
        // for (int i = 0; i < m_xList.length; i++) {
        // System.out.print(m_xList[i] + ", ");
        // }
        // System.out.print("]");
        // System.out.print("\n[");
        // for (int i = 0; i < m_yList.length; i++) {
        // System.out.print(m_yList[i] + ", ");
        // }
        // System.out.print("]");
        // return false;
    }

    public static double[][] getPointJsonData(String pathName) {
        File m_pointFile = new File(pathName);
        try {
            double[][] returnable;
            try (Scanner m_pointScanner = new Scanner(m_pointFile)) {
                String m_pointString = m_pointScanner.useDelimiter("\\Z").next();
                Object m_pointDesralized = Jsoner.deserialize(m_pointString);
                JsonObject m_pointJSONObject = (JsonObject) (m_pointDesralized);
                JsonArray m_points = (JsonArray) m_pointJSONObject.get("points");
                returnable = new double[m_points.size()][dataTypeNum(m_points) + 3];
                for (int i = 0; i < m_points.size(); i++) {
                    returnable[i][0] = getArrayData(m_points, i, "id") == -1 ? (double) i
                            : getArrayData(m_points, i, "id");
                    returnable[i][1] = getArrayData(m_points, i, "x");
                    returnable[i][2] = getArrayData(m_points, i, "y");
                    for (int j = 0; j < getArrayList(m_points, i).length; j++) {
                        returnable[i][j + 3] = getArrayList(m_points, i)[j];
                    }
                }
            } catch (Exception e) {
                System.out.println("Something went wrong! D: (PointHandler.init) (Nested)");
                e.printStackTrace();
                return new double[0][0];
            }

            if (isTriangulable(returnable))
                return returnable;
            else {
                System.out.println("Data can't be sent to Poly2Tri: There are points in the same place");
                return new double[0][0];
            }
        } catch (Exception e) {
            System.out.println("Something went wrong! D: (PointHandler.init)");
            e.printStackTrace();
            return new double[0][0];
        }
    }

    // public static List<TriangulationPoint> getTPoints(double[][] dataArray) {
    // List<TriangulationPoint> m_pointList = new ArrayList<>();
    // for (int i = 0; i < dataArray.length; i++) {
    // m_pointList.add(i, new TPoint(dataArray[i][1], dataArray[i][2],
    // dataArray[i][0]));
    // }
    // return m_pointList;
    // }

    public static double[][] flipArgs(double[][] array) {
        double temp;
        for (double[] array1 : array) {
            temp = array1[3];
            array1[3] = array1[1];
            array1[1] = temp;
            temp = array1[4];
            array1[4] = array1[2];
            array1[2] = temp;
        }
        return array;
    }
}
