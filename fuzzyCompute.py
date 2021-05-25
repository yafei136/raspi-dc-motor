'''
A fuzzy controller for a certain motor.
'''

import numpy as np
import skfuzzy as fuzz
import skfuzzy.control as ctrl

kp_init = 0.02
ki_init = 0.0
kd_init = -0.001

e_range=np.arange(-3,4,1,np.float32)
ec_range=np.arange(-3,4,1,np.float32)
kp_range=np.arange(-0.015,0.02,0.005,np.float32)
ki_range=np.arange(-0.00012,0.00016,0.00004,np.float32)
kd_range=np.arange(-0.006,0.008,0.002,np.float32)

e=ctrl.Antecedent(e_range, 'e')
ec=ctrl.Antecedent(ec_range, 'ec')
kp=ctrl.Consequent(kp_range, 'kp')
ki=ctrl.Consequent(ki_range, 'ki')
kd=ctrl.Consequent(kd_range, 'kd')

e['NB']=fuzz.zmf(e_range,-3,-1)
e['NM']=fuzz.trimf(e_range,[-3,-2,0])
e['NS']=fuzz.trimf(e_range,[-3,-1,1])
e['Z']=fuzz.trimf(e_range,[-2,0,2])
e['PS']=fuzz.trimf(e_range,[-1,1,3])
e['PM']=fuzz.trimf(e_range,[0,2,3])
e['PB']=fuzz.smf(e_range,1,3)

ec['NB']=fuzz.zmf(ec_range,-3,-1)
ec['NM']=fuzz.trimf(ec_range,[-3,-2,0])
ec['NS']=fuzz.trimf(ec_range,[-3,-1,1])
ec['Z']=fuzz.trimf(ec_range,[-2,0,2])
ec['PS']=fuzz.trimf(ec_range,[-1,1,3])
ec['PM']=fuzz.trimf(ec_range,[0,2,3])
ec['PB']=fuzz.smf(ec_range,1,3)

kpp=0.05
kp['NB']=fuzz.zmf(kp_range,-0.3*kpp,0.3*kpp)
kp['NM']=fuzz.trimf(kp_range,[-0.3*kpp,-0.2*kpp,0*kpp])
kp['NS']=fuzz.trimf(kp_range,[-0.3*kpp,-0.1*kpp,0.1*kpp])
kp['Z']=fuzz.trimf(kp_range,[-0.2*kpp,0*kpp,0.2*kpp])
kp['PS']=fuzz.trimf(kp_range,[-0.1*kpp,0.1*kpp,0.3*kpp])
kp['PM']=fuzz.trimf(kp_range,[0*kpp,0.2*kpp,0.3*kpp])
kp['PB']=fuzz.smf(kp_range,0.1*kpp,0.3*kpp)

kii=0.002
ki['NB']=fuzz.zmf(ki_range,-0.06*kii,-0.02*kii)
ki['NM']=fuzz.trimf(ki_range,[-0.06*kii,-0.04*kii,0*kii])
ki['NS']=fuzz.trimf(ki_range,[-0.06*kii,-0.02*kii,0.02*kii])
ki['Z']=fuzz.trimf(ki_range,[-0.04*kii,0*kii,0.04*kii])
ki['PS']=fuzz.trimf(ki_range,[-0.02*kii,0.02*kii,0.06*kii])
ki['PM']=fuzz.trimf(ki_range,[0*kii,0.04*kii,0.06*kii])
ki['PB']=fuzz.smf(ki_range,0.02*kii,0.06*kii)

kdd=0.002
kd['NB']=fuzz.zmf(kd_range,-3*kdd,-1*kdd)
kd['NM']=fuzz.trimf(kd_range,[-3*kdd,-2*kdd,0*kdd])
kd['NS']=fuzz.trimf(kd_range,[-3*kdd,-1*kdd,1*kdd])
kd['Z']=fuzz.trimf(kd_range,[-2*kdd,0*kdd,2*kdd])
kd['PS']=fuzz.trimf(kd_range,[-1*kdd,1*kdd,3*kdd])
kd['PM']=fuzz.trimf(kd_range,[0*kdd,2*kdd,3*kdd])
kd['PB']=fuzz.smf(kd_range,1*kdd,3*kdd)

#质心解模糊
kp.defuzzify_method='centroid'
ki.defuzzify_method='centroid'
kd.defuzzify_method='centroid'

#kp输出为NB的规则
kpRule1 = ctrl.Rule(antecedent=((e['PB'] & ec['PB']) |
                              (e['PM'] & ec['PB']) |
                              (e['PB'] & ec['PM']) ),
                 consequent=kp['NB'], label='rule kpNB')
#kp输出为NM的规则
kpRule2 = ctrl.Rule(antecedent=((e['Z'] & ec['PM']) |
                              (e['Z'] & ec['PB']) |
                              (e['PS'] & ec['PM']) |
                              (e['PS'] & ec['PB']) |
                              (e['PM'] & ec['Z']) |
                              (e['PM'] & ec['PS']) |
                              (e['PM'] & ec['PM']) |
                              (e['PB'] & ec['NS']) |
                              (e['PB'] & ec['Z']) |
                              (e['PB'] & ec['PS']) ),
                 consequent=kp['NM'], label='rule kpNM')

#kp输出为NS的规则
kpRule3 = ctrl.Rule(antecedent=((e['NM'] & ec['PB']) |
                              (e['NS'] & ec['PM']) |
                              (e['NS'] & ec['PB']) |
                              (e['Z'] & ec['PS']) |
                              (e['PS'] & ec['Z']) |
                              (e['PS'] & ec['PS']) |
                              (e['PM'] & ec['NS']) ),
                 consequent=kp['NS'], label='rule kpNS')
#kp输出为Z的规则
kpRule4 = ctrl.Rule(antecedent=((e['NB'] & ec['PM']) |
                              (e['NB'] & ec['PB']) |
                              (e['NM'] & ec['PM']) |
                              (e['NS'] & ec['PS']) |
                              (e['Z'] & ec['Z']) |
                              (e['PS'] & ec['NS']) |
                              (e['PM'] & ec['NM']) |
                              (e['PB'] & ec['NB']) |
                              (e['PB'] & ec['NM']) ),
                 consequent=kp['Z'], label='rule kpZ')
#kp输出为PS的规则
kpRule5 = ctrl.Rule(antecedent=((e['NB'] & ec['PS']) |
                              (e['NM'] & ec['Z']) |
                              (e['NM'] & ec['PS']) |
                              (e['NS'] & ec['Z']) |
                              (e['Z'] & ec['NS']) |
                              (e['PS'] & ec['NB']) |
                              (e['PS'] & ec['NM']) |
                              (e['PM'] & ec['NB']) ),
                 consequent=kp['PS'], label='rule kpPS')
#kp输出为PM的规则
kpRule6 = ctrl.Rule(antecedent=((e['NB'] & ec['NS']) |
                              (e['NB'] & ec['Z']) |
                              (e['NM'] & ec['NS']) |
                              (e['NS'] & ec['NB']) |
                              (e['NS'] & ec['NM']) |
                              (e['NS'] & ec['NS']) |
                              (e['Z'] & ec['NB']) |
                              (e['Z'] & ec['NM']) ),
                 consequent=kp['PM'], label='rule kpPM')
#kp输出为PB的规则
kpRule7 = ctrl.Rule(antecedent=((e['NB'] & ec['NB']) |
                              (e['NB'] & ec['NM']) |
                              (e['NM'] & ec['NB']) |
                              (e['NM'] & ec['NM']) ),
                 consequent=kp['PB'], label='rule kpPB')



#ki规则
#ki输出为NB的规则
kiRule1 = ctrl.Rule(antecedent=((e['NB'] & ec['NB']) |
                              (e['NB'] & ec['NM']) |
                              (e['NM'] & ec['NB']) |
                              (e['NM'] & ec['NM']) |
                              (e['NS'] & ec['NB']) ),
                 consequent=ki['NB'], label='rule kiNB')
#ki输出为NM的规则
kiRule2 = ctrl.Rule(antecedent=((e['NB'] & ec['NS']) |
                              (e['NB'] & ec['Z']) |
                              (e['NM'] & ec['NS']) |
                              (e['NS'] & ec['NM']) |
                              (e['Z'] & ec['NB']) |
                              (e['Z'] & ec['NM']) |
                              (e['PS'] & ec['NB']) ),
                 consequent=ki['NM'], label='rule kiNM')
#ki输出为NS的规则
kiRule3 = ctrl.Rule(antecedent=((e['NB'] & ec['PS']) |
                              (e['NM'] & ec['Z']) |
                              (e['NM'] & ec['PS']) |
                              (e['NS'] & ec['NS']) |
                              (e['NS'] & ec['Z']) |
                              (e['Z'] & ec['NS']) |
                              (e['PS'] & ec['NM']) ),
                 consequent=ki['NS'], label='rule kiNS')
#ki输出为Z的规则
kiRule4 = ctrl.Rule(antecedent=((e['NB'] & ec['PM']) |
                              (e['NB'] & ec['PB']) |
                              (e['NM'] & ec['PM']) |
                              (e['NM'] & ec['PB']) |
                              (e['NS'] & ec['PS']) |
                              (e['Z'] & ec['Z']) |
                              (e['PS'] & ec['NS']) |
                              (e['PM'] & ec['NB']) |
                              (e['PM'] & ec['NM']) |
                              (e['PB'] & ec['NB']) |
                              (e['PB'] & ec['NM']) ),
                 consequent=ki['Z'], label='rule kiZ')
#ki输出为PS的规则
kiRule5 = ctrl.Rule(antecedent=((e['NS'] & ec['PM']) |
                              (e['NS'] & ec['PB']) |
                              (e['Z'] & ec['PS']) |
                              (e['PS'] & ec['Z']) |
                              (e['PS'] & ec['PS']) |
                              (e['PM'] & ec['NS']) |
                              (e['PM'] & ec['Z']) |
                              (e['PB'] & ec['NS']) ),
                 consequent=ki['PS'], label='rule kiPS')
#ki输出为PM的规则
kiRule6 = ctrl.Rule(antecedent=((e['Z'] & ec['PM']) |
                              (e['Z'] & ec['PB']) |
                              (e['PS'] & ec['PM']) |
                              (e['PM'] & ec['PS']) |
                              (e['PB'] & ec['Z']) |
                              (e['PB'] & ec['PS']) ),
                 consequent=ki['PM'], label='rule kiPM')
#ki输出为PB的规则
kiRule7 = ctrl.Rule(antecedent=((e['PS'] & ec['PB']) |
                              (e['PM'] & ec['PM']) |
                              (e['PM'] & ec['PB']) |
                              (e['PB'] & ec['PM']) |
                              (e['PB'] & ec['PB']) ),
                 consequent=ki['PB'], label='rule kiPB')



#kd规则
#kd输出为NB的规则
kdRule1 = ctrl.Rule(antecedent=((e['NB'] & ec['NS']) |
                              (e['NB'] & ec['Z']) |
                              (e['NB'] & ec['PS']) |
                              (e['NM'] & ec['NS']) ),
                 consequent=kd['NB'], label='rule kdNB')
#kd输出为NM的规则
kdRule2 = ctrl.Rule(antecedent=((e['NB'] & ec['PM']) |
                              (e['NM'] & ec['Z']) |
                              (e['NM'] & ec['PS']) |
                              (e['NS'] & ec['NS']) |
                              (e['NS'] & ec['Z']) ),
                 consequent=kd['NM'], label='rule kdNM')
#kd输出为NS的规则
kdRule3 = ctrl.Rule(antecedent=((e['NB'] & ec['NM']) |
                              (e['NM'] & ec['NM']) |
                              (e['NM'] & ec['PM']) |
                              (e['NS'] & ec['NM']) |
                              (e['NS'] & ec['PS']) |
                              (e['NS'] & ec['PM']) |
                              (e['Z'] & ec['NM']) |
                              (e['Z'] & ec['NS']) |
                              (e['Z'] & ec['Z']) |
                              (e['Z'] & ec['PS']) |
                              (e['Z'] & ec['PM']) |
                              (e['PM'] & ec['NM']) ),
                 consequent=kd['NS'], label='rule kdNS')
#kd输出为Z的规则
kdRule4 = ctrl.Rule(antecedent=((e['NM'] & ec['PB']) |
                              (e['NS'] & ec['NB']) |
                              (e['NS'] & ec['PB']) |
                              (e['Z'] & ec['NB']) |
                              (e['Z'] & ec['PB']) |
                              (e['PS'] & ec['NB']) |
                              (e['PS'] & ec['NM']) |
                              (e['PS'] & ec['NS']) |
                              (e['PS'] & ec['Z']) |
                              (e['PS'] & ec['PS']) |
                              (e['PS'] & ec['PM']) |
                              (e['PS'] & ec['PB']) ),
                 consequent=kd['Z'], label='rule kdZ')
#kd输出为PS的规则
kdRule5 = ctrl.Rule(antecedent=((e['NB'] & ec['NB']) |
                              (e['NB'] & ec['PB']) |
                              (e['NM'] & ec['NB']) |
                              (e['PM'] & ec['NS']) |
                              (e['PM'] & ec['Z']) |
                              (e['PM'] & ec['PS']) |
                              (e['PM'] & ec['PM']) |
                              (e['PB'] & ec['PS']) |
                              (e['PB'] & ec['PM']) ),
                 consequent=kd['PS'], label='rule kdPS')
#kd输出为PM的规则
kdRule6 = ctrl.Rule(antecedent=((e['PB'] & ec['NM']) |
                              (e['PB'] & ec['NS']) |
                              (e['PB'] & ec['Z']) ),
                 consequent=kd['PM'], label='rule kdkp_initPM')
#kd输出为PB的规则
kdRule7 = ctrl.Rule(antecedent=((e['PM'] & ec['NB']) |
                              (e['PM'] & ec['PB']) |
                              (e['PB'] & ec['NB']) |
                              (e['PB'] & ec['PB']) ),
                 consequent=kd['PB'], label='rule kdPB')

# 系统和运行环境初始化
system = ctrl.ControlSystem(rules=[kpRule1,kpRule2,kpRule3,kpRule4,kpRule5,kpRule6,kpRule7,kiRule1,kiRule2,kiRule3,kiRule4,kiRule5,kiRule6,kiRule7,kdRule1,kdRule2,kdRule3,kdRule4,kdRule5,kdRule6,kdRule7])
sim = ctrl.ControlSystemSimulation(system)

def fuzzyCompute(e,ec):
    sim.input['e'] = e
    sim.input['ec'] = ec
    sim.compute()   # 运行系统
    output_kp = sim.output['kp']
    output_ki = sim.output['ki']
    output_kd = sim.output['kd']
    return(kp_init+output_kp,ki_init+output_ki,kd_init+output_kd)
