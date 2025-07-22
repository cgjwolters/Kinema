using KinemaLibCs;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Xml.Linq;

namespace KinemaLibCs
{
  public partial class Model
  {
    const int CoachSz = 7;

    static Body DefineBogiePair(Model model, int idx,
                                ref readonly ArcLinTrack leftTrk, ref readonly ArcLinTrack rightTrk,
                                bool frontMost)
    {
      Body ground = model.BodyMap["Ground"];

      const double wheelRad = 0.20565;
      const double coachDist = 2.5;

      Trf3 unitpos = new();
      Trf3 pos, pos2;

      double axPos = idx * coachDist;
      if (frontMost) {
        axPos -= 0.37;
      }

      // Left side:

      pos = new(new(-0.55, -0.2125 + axPos, 0.625), new(-1, 0, 0), new(0, 0, 1));
      Body wheelLeftR = new(model, cat("WheelLeftR", idx), pos);

      Grip trkLeftR = new(model, cat("TrackLeftR", idx), ground, unitpos, wheelLeftR, unitpos);
      JointTrack jntLeftR = new(trkLeftR, cat("JntLeftR", idx), leftTrk, wheelRad);

      pos = new(new(-0.55, 0.2125 + axPos, 0.625), new(-1, 0, 0), new(0, 0, 1));
      Body wheelLeftF = new(model, cat("WheelLeftF", idx), pos);

      Grip trkLeftF = new(model, cat("TrackLeftF", idx), ground, unitpos, wheelLeftF, unitpos);
      JointTrack jntLeftF = new(trkLeftF, cat("JntLeftF", idx), leftTrk, wheelRad);

      pos = new Trf3(new(-0.55, axPos, 0.1855), new(-1, 0, 0), new(0, 0, 1));
      Body bogieLeft = new(model, cat("bogieLeft", idx), pos);

      pos = new Trf3(new(0.4395, -0.2125, 0), new(0, 0, 1), new(1, 0, 0));
      Grip axleLeftR = new(model, cat("AxleLeftR", idx), wheelLeftR, unitpos, bogieLeft, pos);
      JointRev jAxleLeftR = new(axleLeftR, cat("JAxleLeftR", idx));

      pos = new(new(0.4395, 0.2125, 0), new(0, 0, 1), new(1, 0, 0));
      Grip axleLeftF = new(model, cat("AxleLeftF", idx), wheelLeftF, unitpos, bogieLeft, pos);
      JointRev jAxleLeftF = new(axleLeftF, cat("JAxleLeftF", idx));


      pos = new Trf3(new(-0.55, axPos, 0), new(0, 0, 1), new(1, 0, 0));
      Body columnLeft = new(model, cat("ColumnLeft", idx), pos);

      pos = new Trf3(new(0, 0, 0.1855), new(-1, 0, 0), new(0, 0, 1));
      Grip cantiLeft = new(model, cat("CantiLeft", idx), columnLeft, pos, bogieLeft, unitpos);
      JointRev jCantiLeft = new(cantiLeft, cat("JCantiLeft", idx));

      // Right side:

      pos = new Trf3(new(0.55, -0.2125 + axPos, 0.625), new(-1, 0, 0), new(0, 0, 1));
      Body wheelRightR = new(model, cat("WheelRightR", idx), pos);

      Grip trkRightR = new(model, cat("TrackRightR", idx), ground, unitpos, wheelRightR, unitpos);
      JointTrack jntRightR = new(trkRightR, cat("JntRightR", idx), rightTrk, wheelRad);

      pos = new Trf3(new(0.55, 0.2125 + axPos, 0.625), new(-1, 0, 0), new(0, 0, 1));
      Body wheelRightF = new(model, cat("WheelRightF", idx), pos);

      Grip trkRightF = new(model, cat("TrackRightF", idx), ground, unitpos, wheelRightF, unitpos);
      JointTrack jntRightF = new(trkRightF, cat("JntRightF", idx), rightTrk, wheelRad);

      pos = new Trf3(new(0.55, axPos, 0.1855), new(-1, 0, 0), new(0, 0, 1));
      Body bogieRight = new(model, cat("BogieRight", idx), pos);

      pos = new Trf3(new(0.4395, -0.2125, 0.0), new(0, 0, 1), new(1, 0, 0));
      Grip axleRightR = new(model, cat("AxleRightR", idx), wheelRightR, unitpos, bogieRight, pos);
      JointRev jAxleRightR = new(axleRightR, cat("JAxleRightR", idx));

      pos = new Trf3(new(0.4395, 0.2125, 0.0), new(0, 0, 1), new(1, 0, 0));
      Grip axleRightF = new(model, cat("AxleRightF", idx), wheelRightF, unitpos, bogieRight, pos);
      JointRev jAxleRightF = new(axleRightF, cat("JAxleRightF", idx));

      pos = new Trf3(new(0.55, axPos, 0), new(0, 0, 1), new(1, 0, 0));
      Body columnRight = new(model, cat("ColumnRight", idx), pos);

      pos = new Trf3(new(0, 0, 0.1855), new(-1, 0, 0), new(0, 0, 1));
      Grip cantiRight = new(model, cat("CantiRight", idx), columnRight, pos, bogieRight, unitpos);

      // Coach

      pos = new Trf3(new(0.0, axPos, 0.0), new(0, 0, 1), new(1, 0, 0));
      Body coach = new(model, cat("Coach", idx), pos);

      pos = new Trf3(new(-0.55, 0, 0), new(0, 0, 1), new(1, 0, 0));
      Grip coachLeft = new(model, cat("CoachLeft", idx), coach, pos, columnLeft, unitpos);
      JointRev jCoachLeft = new(coachLeft, cat("JCoachLeft", idx));

      pos = new Trf3(new(0.55, 0.0, 0), new(0, 0, 1), new(1, 0, 0));
      Grip coachRight = new(model, cat("CoachRight", idx), coach, pos, columnRight, unitpos);
      JointRev jCoachRight = new(coachRight, cat("JCoachRight", idx));

      if (frontMost) {
        JointRev CantiRight = new(cantiRight, cat("JCantiRight", idx));

        // SteerBar

        pos = new Trf3(new(0.0, axPos + 0.185, 0), new(0, 0, 1), new(1, 0, 0));
        Body steerBar = new(model, cat("SteerBar", idx), pos);

        pos = new Trf3(new(-0.040, 0.185, 0), new(0, 0, 1), new(1, 0, 0));
        pos2 = new Trf3(new(-0.55 - 0.040, 0, 0), new(0, 0, 1), new(1, 0, 0));
        Grip steerBarLeft = new(model, cat("SteerBarLeft", idx), columnLeft, pos, steerBar, pos2);
        JointBall jSteerBarLeft = new(steerBarLeft, cat("JBallLeft", idx));

        pos = new Trf3(new(0.040, 0.185, 0), new(0, 0, 1), new(1, 0, 0));
        pos2 = new Trf3(new(0.55 + 0.040, 0, 0), new(0, 0, 1), new(1, 0, 0));
        Grip steerBarRight = new(model, cat("SteerBarRight", idx), columnRight, pos, steerBar, pos2);
        JointCross jSteerBarRight = new(steerBarRight, cat("JBallRight", idx));
      }
      else {
        JointRevSlide jCantiRight = new(cantiRight, cat("JCantiRight", idx));
      }


      new Probe(coach, cat("AxleMidPt", idx), unitpos);

      return coach;
    }

    //--------------------------------------------------------------------------------------
    //---- Defines the complete kinematic model --------------------------------------------
    //--------------------------------------------------------------------------------------

    bool DefineModel(in ArcLinTrack leftTrk, in ArcLinTrack rightTrk)
    {
      Trf3 unitpos = new();
      Trf3 pos;

      Body ground = new(this, "Ground", unitpos);

      for (int i = 0; i < CoachSz; ++i) {
        Body coach = DefineBogiePair(this, i, in leftTrk, in rightTrk, i == CoachSz - 1);
      }

      for (int i = 1; i < CoachSz; ++i) {
        Body prevCoach = BodyMap[cat("Coach", i - 1)];
        Body coach = BodyMap[cat("Coach", i)];

        Trf3 pos2;

        if (i < CoachSz - 1) {
          pos = new Trf3(new Vec3(0, 2.5 - 0.185, 0), new Vec3(0, 0, 1), new Vec3(1, 0, 0));
          pos2 = new Trf3(new Vec3(0, -0.185, 0), new Vec3(0, 0, 1), new Vec3(1, 0, 0));
          Grip gCoach = new Grip(this, cat("GCoach", i - 1), prevCoach, pos, coach, pos2);
          JointBall jCoach = new(gCoach, cat("JCoach", i - 1));
        }
        else {
          pos = new Trf3(new Vec3(0, 2.13, 0), new Vec3(0, 1, 0), new Vec3(1, 0, 0));
          pos2 = new Trf3(new Vec3(0, 0, 0), new Vec3(0, 1, 0), new Vec3(1, 0, 0));
          Grip gCoach = new Grip(this, cat("GCoach", i - 1), prevCoach, pos, coach, pos2);
          JointRev jCoach = new(gCoach, cat("JCoach", i - 1));
        }
      }

      // No sideways slide

      SetFixedAll(false);

      AbstractJoint jnt;

      for (int i = 0; i < CoachSz; ++i) {
        jnt = JointMap["JntLeftR" + i];
        if (jnt is null) {
          Logger.WriteMsg("Pointer Error 3!");
          return false;
        }
        jnt.SetFixed(3, true);

        jnt = JointMap["JntLeftF" + i];
        if (jnt is null) {
          Logger.WriteMsg("Pointer Error 4!");
          return false;
        }
        jnt.SetFixed(3, true);

        jnt = JointMap["JntRightR" + i];
        if (jnt is null) {
          Logger.WriteMsg("Pointer Error 5!");
          return false;
        }
        jnt.SetFixed(3, i < CoachSz - 1);

        jnt = JointMap["JntRightF" + i];
        if (jnt is null) {
          Logger.WriteMsg("Pointer Error 6!");
          return false;
        }
        jnt.SetFixed(3, i < CoachSz - 1);
      }

      jnt = JointMap["JntLeftR0"];
      jnt.SetFixed(0, true); // Drive wheel

      return true;
    }

    void PutOnTrack(double startOffset, in ArcLinTrack leftTrk, in ArcLinTrack righTrk)
    {
      Vec3 p1, yDir, p2;

      leftTrk.GetPointAndDir(startOffset, out p1, out yDir);
      righTrk.GetPoint(startOffset, out p2);

      Vec3 org = new(p1); org += p2; org /= 2.0; org.z += 0.0;

      Vec3 xDir = new(p2); xDir -= p1; xDir.UnitLen3();

      yDir.UnitLen3();

      Vec3 zDir = new(xDir.Outer(yDir));
      zDir.UnitLen3();

      Trf3 trf = new(org, zDir, xDir); // Rotate

      Transform(trf);
    }

    bool SetWheel(string name, double val)
    {
      AbstractJoint jnt = JointMap[name];

      if (jnt == null) {
        Logger.WriteMsg("Pointer Error!");
        return false;
      }

      jnt.SetVal(0, val);

      return true;
    }

    bool SetWheelVars(double startOffset)
    {
      for (int i = 0; i < CoachSz; ++i) {
        double off = i * 2.5 + startOffset;
        if (i == CoachSz - 1) off -= 0.37;

        if (!SetWheel("JntLeftR" + i, off - 0.2125)) return false;
        if (!SetWheel("JntLeftF" + i, off + 0.2125)) return false;
        if (!SetWheel("JntRightR" + i, off - 0.2125)) return false;
        if (!SetWheel("JntRightF" + i, off + 0.2125)) return false;
      }

      return true;
    }

    bool ResetFixedVars()
    {
      // Sideways slide on all wheels is zero:

      for (int i = 0; i < CoachSz; ++i) {
        string name = "JntLeftR" + i;

        AbstractJoint jnt = JointMap[name];
        if (jnt == null) {
          Logger.WriteMsg("Pointer Error!");
          return false;
        }

        jnt.SetVal(3, 0.0);

        name = "JntLeftF" + i;

        jnt = JointMap[name];
        if (jnt == null) {
          Logger.WriteMsg("Pointer Error!");
          return false;
        }

        jnt.SetVal(3, 0.0);

        name = "JntRightR" + i;

        jnt = JointMap[name];
        if (jnt == null) {
          Logger.WriteMsg("Pointer Error!");
          return false;
        }

        jnt.SetVal(3, 0.0);

        name = "JntRightF" + i;

        jnt = JointMap[name];
        if (jnt == null) {
          Logger.WriteMsg("Pointer Error!");
          return false;
        }

        jnt.SetVal(3, 0.0);
      }

      return true;
    }
  }
}