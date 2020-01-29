import com.neuronrobotics.bowlerstudio.creature.ICadGenerator;
import com.neuronrobotics.bowlerstudio.creature.CreatureLab
import com.neuronrobotics.bowlerstudio.physics.TransformFactory
import com.neuronrobotics.sdk.addons.kinematics.DHParameterKinematics
import com.neuronrobotics.sdk.addons.kinematics.MobileBase
import com.neuronrobotics.sdk.addons.kinematics.math.RotationNR
import eu.mihosoft.vrl.v3d.CSG;
import org.apache.commons.io.IOUtils;
import com.neuronrobotics.bowlerstudio.vitamins.*;
import java.nio.file.Paths;
import eu.mihosoft.vrl.v3d.FileUtil;
import com.neuronrobotics.bowlerstudio.vitamins.*;
import javafx.scene.transform.Affine;
import com.neuronrobotics.sdk.addons.kinematics.LinkConfiguration
import com.neuronrobotics.sdk.addons.kinematics.DHLink
import com.neuronrobotics.sdk.addons.kinematics.math.TransformNR

CSG reverseDHValues(CSG incoming, DHLink dh){
	println "Reversing "+dh
	TransformNR step = new TransformNR(dh.DhStep(0))
	Transform move = com.neuronrobotics.bowlerstudio.physics.TransformFactory.nrToCSG(step)
	return incoming.transformed(move)
}

CSG moveDHValues(CSG incoming, DHLink dh){
	TransformNR step = new TransformNR(dh.DhStep(0)).inverse()
	Transform move = com.neuronrobotics.bowlerstudio.physics.TransformFactory.nrToCSG(step)
	return incoming.transformed(move)

}

class MyCadGen implements ICadGenerator {

    @Override
    ArrayList<CSG> generateBody(MobileBase mobileBase) {
        CSG body  = new Cube(30).toCSG()
        body.setColor(javafx.scene.paint.Color.WHITE)
        body.setManipulator(b.getRootListener());
        return [body];
    }

    @Override
    ArrayList<CSG> generateCad(DHParameterKinematics d, int linkIndex) {
        println("Generating CAD for " + d + " linkIndex=" + linkIndex)
        def vitaminLocations = new HashMap<TransformNR, ArrayList<String>>()

        ArrayList<DHLink> dhLinks = d.getChain().getLinks()
        ArrayList<CSG> allCad=new ArrayList<CSG>()
        int i=linkIndex;
        DHLink dh = dhLinks.get(linkIndex)
        // Hardware to engineering units configuration
        LinkConfiguration conf = d.getLinkConfiguration(i);
        // Engineering units to kinematics link (limits and hardware type abstraction)
        AbstractLink abstractLink = d.getAbstractLink(i) as AbstractLink;
        // Transform used by the UI to render the location of the object
        Affine manipulator = dh.getListener();
        // loading the vitamins referenced in the configuration
//        CSG servo = Vitamins.get(conf.getElectroMechanicalType(),conf.getElectroMechanicalSize())


        TransformNR locationOfMotorMount = new TransformNR(dh.DhStep(0)).inverse()
        vitaminLocations.put(
                locationOfMotorMount,
                [conf.getElectroMechanicalType(), conf.getElectroMechanicalSize()] as ArrayList<String>
        )



//        CSG tmpSrv = moveDHValues(servo,dh)
//
//        //Compute the location of the base of this limb to place objects at the root of the limb
//        TransformNR step = d.getRobotToFiducialTransform()
//        Transform locationOfBaseOfLimb = TransformFactory.nrToCSG(step)


        double totalMassKg = 0.0
        TransformNR centerOfMassFromCentroid = new TransformNR()
        def intermediateCoMs = new ArrayList<TransformNR>()

        for (TransformNR vitaminLocation : vitaminLocations.keySet()) {
            def vitaminType = vitaminLocations.get(vitaminLocation)[0]
            def vitaminSize = vitaminLocations.get(vitaminLocation)[1]
            def vitaminData = Vitamins.getConfiguration(vitaminType, vitaminSize)
            def vitaminCad = Vitamins.get(vitaminType, vitaminSize)
            def massKg = vitaminData["massKg"] as double
            def comCentroid = vitaminLocation.times(
                    new TransformNR(
                            vitaminData["massCentroidX"] as double,
                            vitaminData["massCentroidY"] as double,
                            vitaminData["massCentroidZ"] as double,
                            new RotationNR()
                    )
            )

            allCad.add(vitaminCad.transformed(TransformFactory.nrToCSG(vitaminLocation)))
            totalMassKg += massKg
            intermediateCoMs.add(comCentroid)
            centerOfMassFromCentroid.x = (centerOfMassFromCentroid.x + comCentroid.x) * massKg
            centerOfMassFromCentroid.y = (centerOfMassFromCentroid.y + comCentroid.y) * massKg
            centerOfMassFromCentroid.z = (centerOfMassFromCentroid.z + comCentroid.z) * massKg
        }

        centerOfMassFromCentroid.x = centerOfMassFromCentroid.x / totalMassKg
        centerOfMassFromCentroid.y = centerOfMassFromCentroid.y / totalMassKg
        centerOfMassFromCentroid.z = centerOfMassFromCentroid.z / totalMassKg

        conf.setMassKg(totalMassKg)
        conf.setCenterOfMassFromCentroid(centerOfMassFromCentroid)
        println("Computed totalMassKg=" + totalMassKg)
        println("Computed centerOfMassFromCentroid=" + centerOfMassFromCentroid)


        tmpSrv.setManipulator(manipulator)
        allCad.add(tmpSrv)
        println "Generating link: "+linkIndex

        if (i == 0) {
            // more at https://github.com/NeuronRobotics/java-bowler/blob/development/src/main/java/com/neuronrobotics/sdk/addons/kinematics/DHLink.java
            println dh
            println "D = "+dh.getD()// this is the height of the link
            println "R = "+dh.getR()// this is the radius of rotation of the link
            println "Alpha = "+Math.toDegrees(dh.getAlpha())// this is the alpha rotation
            println "Theta = "+Math.toDegrees(dh.getTheta())// this is the rotation about hte final like orentation
            println conf // gets the link hardware map from https://github.com/NeuronRobotics/java-bowler/blob/development/src/main/java/com/neuronrobotics/sdk/addons/kinematics/LinkConfiguration.java
            println conf.getHardwareIndex() // gets the link hardware index
            println conf.getScale() // gets the link hardware scale to degrees from link units
            // more from https://github.com/NeuronRobotics/java-bowler/blob/development/src/main/java/com/neuronrobotics/sdk/addons/kinematics/AbstractLink.java
            println  "Max engineering units for link = " + abstractLink.getMaxEngineeringUnits()
            println  "Min engineering units for link = " + abstractLink.getMinEngineeringUnits()
            println "Position "+abstractLink.getCurrentEngineeringUnits()
            println manipulator
        }

        return allCad;
    }
}

return new MyCadGen()
