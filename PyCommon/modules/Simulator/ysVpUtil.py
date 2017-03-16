def align2D(vpModel):
    lv = vpModel.getBodyVelocityGlobal(0)
    vpModel.setBodyVelocityGlobal(0, (lv[0], lv[1], 0.))
    av = vpModel.getBodyAngVelocityGlobal(0)
    vpModel.setBodyAngVelocityGlobal(0, (av[0], 0., av[2]))
