package frc.robot;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

import com.github.cliftonlabs.json_simple.JsonArray;
import com.github.cliftonlabs.json_simple.JsonObject;
import com.github.cliftonlabs.json_simple.Jsoner;

public class Output {
    private static double[][] m_dataArray;

    private static int[] getArrayInts(JsonArray array) {
        int[] output = new int[3];
        for (int i = 0; i < 3; i++) {
            output[i] = array.getInteger(i);
        }
        return output;
    }

    public static List<OutputTriangle> getJSONTriangles(String pathName, String pointFileName) {
        m_dataArray = PointHandler.getPointJsonData(pointFileName);
        List<OutputTriangle> m_dataTriangles = new ArrayList<OutputTriangle>();
        File m_triFile = new File(pathName);
        try (Scanner m_triScanner = new Scanner(m_triFile)) {
            String m_triString = m_triScanner.useDelimiter("\\Z").next();
            Object m_triDel = Jsoner.deserialize(m_triString);
            JsonObject m_triFileObject = (JsonObject) (m_triDel);
            JsonArray m_Jsontriangles = (JsonArray) (m_triFileObject.get("triangles"));
            JsonObject m_arrayObject = new JsonObject();
            OutputTriangle m_iterTri = new OutputTriangle();
            int[] m_arrayPointData = new int[3];
            int[] m_arrayNeighborData = new int[3];

            for (int i = 0; i < m_Jsontriangles.size(); i++) {
                m_arrayObject = (JsonObject) m_Jsontriangles.get(i);
                m_arrayPointData = getArrayInts((JsonArray) m_arrayObject.get("points"));
                m_dataTriangles.add(new OutputTriangle(m_dataArray, m_arrayPointData));
            }

            for (int i = 0; i < m_dataTriangles.size(); i++) {
                m_iterTri = m_dataTriangles.get(i);
                m_arrayObject = (JsonObject) m_Jsontriangles.get(i);
                m_arrayNeighborData = getArrayInts((JsonArray) m_arrayObject.get("neighbors"));
                m_iterTri.setNeighbor(1, m_arrayNeighborData[0] == -1 ? new OutputTriangle()
                        : m_dataTriangles.get(m_arrayNeighborData[0]));
                m_iterTri.setNeighbor(2, m_arrayNeighborData[1] == -1 ? new OutputTriangle()
                        : m_dataTriangles.get(m_arrayNeighborData[1]));
                m_iterTri.setNeighbor(3, m_arrayNeighborData[2] == -1 ? new OutputTriangle()
                        : m_dataTriangles.get(m_arrayNeighborData[2]));
            }

        } catch (Exception e) {
            System.out.println("Something broke in getJSONTriangles D:");
            e.printStackTrace();
        }

        return m_dataTriangles;
    }
}
