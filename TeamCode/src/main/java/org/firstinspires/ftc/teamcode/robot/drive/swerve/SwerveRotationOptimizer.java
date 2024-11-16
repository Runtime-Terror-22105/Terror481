//package org.firstinspires.ftc.teamcode.robot.drive.swerve;//package org.firstinspires.ftc.teamcode.drive.swerve;
//import weka.classifiers.functions.Logistic;
//import weka.classifiers.functions.SMOreg;
//import weka.classifiers.trees.RandomForest;
//import weka.core.Instances;
//import weka.core.SerializationHelper;
//import weka.core.converters.ConverterUtils.DataSource;
//import weka.classifiers.trees.J48;
//import weka.classifiers.Evaluation;
//import weka.classifiers.Classifier;
//
//
//import java.util.Random;
//public class SwerveRotationOptimizer {
//    public static void main(String[] args) throws Exception {
//
//    }
//
//    public void writemodel() throws Exception{
//        DataSource source = new DataSource("TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/swerve/SwerveRotationOpt.arff");
//        Instances data = source.getDataSet();
//
//        // Set class index to the last attribute
//        if (data.classIndex() == -1) {
//            data.setClassIndex(data.numAttributes() - 1);
//        }
//
////        RandomForest model = new RandomForest();
////        String[] options = new String[]{"-I", "10"}; // Set number of trees to 100
////        model.setOptions(options);
////        model.buildClassifier(data);
//
//        Logistic model = new Logistic();
//        model.buildClassifier(data);
//
//        // Evaluate the model
//        Evaluation eval = new Evaluation(data);
//        eval.crossValidateModel(model, data, 2, new Random(1));
//        System.out.println(eval.toSummaryString("\nResults\n======\n", false));
//
//        // Save the model
//        SerializationHelper.write("logistic_model.model", model);
//    }
//
//    public void predict_with_model(double WheelSpeed, double angle_delta, double coaxeffect, double drift, double Robot_Speed) throws Exception{
//        Classifier model = (Classifier) SerializationHelper.read("logistic_model.model.model");
//    }
//}
