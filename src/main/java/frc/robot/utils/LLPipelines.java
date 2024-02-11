// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;



/** Add your docs here. */
public class LLPipelines {

    public enum pipelines {
        APRILTAG(0, pipelinetype.fiducialmarkers),
        APRILTAGSTARTRED(1, pipelinetype.fiducialmarkers), // tags 3 and 4 only
        APRILTAGSTARTBLUE(2, pipelinetype.fiducialmarkers), // tags 7 and 8 only
        SPART_3(3, pipelinetype.fiducialmarkers),
        SPARE_4(4, pipelinetype.fiducialmarkers),
        SPARE_5(5, pipelinetype.fiducialmarkers),
        GRIP_6(6, pipelinetype.grip),
        PYTHON_7(7, pipelinetype.python),
        NOTE_DETECT(8, pipelinetype.detector),
        SPARE_DETECT(9, pipelinetype.detector);

        public static final pipelines values[] = values();

        private pipelinetype type;

        public String pipelineTypeName;

        private int number;

        private pipelines(int number, pipelinetype type) {
            this.number = number;
            this.type = type;
        }

    }

    public enum pipelinetype {
        retroreflective,
        grip,
        python,
        fiducialmarkers,
        classifier,
        detector;
    
        public static final pipelinetype values[] = values();
      }

}
