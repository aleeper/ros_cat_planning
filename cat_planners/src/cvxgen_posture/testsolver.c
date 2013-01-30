/* Produced by CVXGEN, 2013-01-29 22:09:07 -0500.  */
/* CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2012 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: testsolver.c. */
/* Description: Basic test harness for solver.c. */
#include "solver.h"
Vars vars;
Params params;
Workspace work;
Settings settings;
#define NUMTESTS 0
int main(int argc, char **argv) {
  int num_iters;
#if (NUMTESTS > 0)
  int i;
  double time;
  double time_per;
#endif
  set_defaults();
  setup_indexing();
  load_default_data();
  /* Solve problem instance for the record. */
  settings.verbose = 1;
  num_iters = solve();
#ifndef ZERO_LIBRARY_MODE
#if (NUMTESTS > 0)
  /* Now solve multiple problem instances for timing purposes. */
  settings.verbose = 0;
  tic();
  for (i = 0; i < NUMTESTS; i++) {
    solve();
  }
  time = tocq();
  printf("Timed %d solves over %.3f seconds.\n", NUMTESTS, time);
  time_per = time / NUMTESTS;
  if (time_per > 1) {
    printf("Actual time taken per solve: %.3g s.\n", time_per);
  } else if (time_per > 1e-3) {
    printf("Actual time taken per solve: %.3g ms.\n", 1e3*time_per);
  } else {
    printf("Actual time taken per solve: %.3g us.\n", 1e6*time_per);
  }
#endif
#endif
  return 0;
}
void load_default_data(void) {
  params.x_d[0] = 0.20319161029830202;
  params.x_d[1] = 0.8325912904724193;
  params.x_d[2] = -0.8363810443482227;
  params.J_v[0] = 0.04331042079065206;
  params.J_v[1] = 1.5717878173906188;
  params.J_v[2] = 1.5851723557337523;
  params.J_v[3] = -1.497658758144655;
  params.J_v[4] = -1.171028487447253;
  params.J_v[5] = -1.7941311867966805;
  params.J_v[6] = -0.23676062539745413;
  params.J_v[7] = -1.8804951564857322;
  params.J_v[8] = -0.17266710242115568;
  params.J_v[9] = 0.596576190459043;
  params.J_v[10] = -0.8860508694080989;
  params.J_v[11] = 0.7050196079205251;
  params.J_v[12] = 0.3634512696654033;
  params.J_v[13] = -1.9040724704913385;
  params.J_v[14] = 0.23541635196352795;
  params.J_v[15] = -0.9629902123701384;
  params.J_v[16] = -0.3395952119597214;
  params.J_v[17] = -0.865899672914725;
  params.J_v[18] = 0.7725516732519853;
  params.J_v[19] = -0.23818512931704205;
  params.J_v[20] = -1.372529046100147;
  params.weight_x[0] = 1.5446490180318446;
  params.weight_x[1] = 1.780314764511367;
  params.weight_x[2] = 1.3063635323761797;
  params.w_d[0] = -1.1121684642712744;
  params.w_d[1] = -0.44811496977740495;
  params.w_d[2] = 1.7455345994417217;
  params.J_w[0] = 1.9039816898917352;
  params.J_w[1] = 0.6895347036512547;
  params.J_w[2] = 1.6113364341535923;
  params.J_w[3] = 1.383003485172717;
  params.J_w[4] = -0.48802383468444344;
  params.J_w[5] = -1.631131964513103;
  params.J_w[6] = 0.6136436100941447;
  params.J_w[7] = 0.2313630495538037;
  params.J_w[8] = -0.5537409477496875;
  params.J_w[9] = -1.0997819806406723;
  params.J_w[10] = -0.3739203344950055;
  params.J_w[11] = -0.12423900520332376;
  params.J_w[12] = -0.923057686995755;
  params.J_w[13] = -0.8328289030982696;
  params.J_w[14] = -0.16925440270808823;
  params.J_w[15] = 1.442135651787706;
  params.J_w[16] = 0.34501161787128565;
  params.J_w[17] = -0.8660485502711608;
  params.J_w[18] = -0.8880899735055947;
  params.J_w[19] = -0.1815116979122129;
  params.J_w[20] = -1.17835862158005;
  params.weight_w[0] = 1.2013787110430731;
  params.weight_w[1] = 1.514035059817442;
  params.weight_w[2] = 1.0872293687808048;
  params.weight_q[0] = 1.4835855323515865;
  params.weight_q[1] = 1.3621762123878334;
  params.weight_q[2] = 1.7076866218156712;
  params.weight_q[3] = 1.7467462231020046;
  params.weight_q[4] = 1.6910929218557644;
  params.weight_q[5] = 1.6891804137549142;
  params.weight_q[6] = 1.3736001241489282;
  params.q[0] = 0.6725392189410702;
  params.q[1] = -0.6406053441727284;
  params.q[2] = 0.29117547947550015;
  params.q[3] = -0.6967713677405021;
  params.q[4] = -0.21941980294587182;
  params.q[5] = -1.753884276680243;
  params.q[6] = -1.0292983112626475;
  params.q_set[0] = 1.8864104246942706;
  params.q_set[1] = -1.077663182579704;
  params.q_set[2] = 0.7659100437893209;
  params.q_set[3] = 0.6019074328549583;
  params.q_set[4] = 0.8957565577499285;
  params.q_set[5] = -0.09964555746227477;
  params.q_set[6] = 0.38665509840745127;
  params.weight_posture[0] = 1.0669694239328265;
  params.weight_posture[1] = 1.0725621378222334;
  params.weight_posture[2] = 1.1989760262970783;
  params.weight_posture[3] = 1.1518609970085412;
  params.weight_posture[4] = 1.1001043445814447;
  params.weight_posture[5] = 1.1292938646088542;
  params.weight_posture[6] = 1.5532777318076536;
  params.normal_0[0] = -1.248740700304487;
  params.normal_0[1] = 1.808404972124833;
  params.normal_0[2] = 0.7264471152297065;
  params.J_c_0[0] = 0.16407869343908477;
  params.J_c_0[1] = 0.8287224032315907;
  params.J_c_0[2] = -0.9444533161899464;
  params.J_c_0[3] = 1.7069027370149112;
  params.J_c_0[4] = 1.3567722311998827;
  params.J_c_0[5] = 0.9052779937121489;
  params.J_c_0[6] = -0.07904017565835986;
  params.J_c_0[7] = 1.3684127435065871;
  params.J_c_0[8] = 0.979009293697437;
  params.J_c_0[9] = 0.6413036255984501;
  params.J_c_0[10] = 1.6559010680237511;
  params.J_c_0[11] = 0.5346622551502991;
  params.J_c_0[12] = -0.5362376605895625;
  params.J_c_0[13] = 0.2113782926017822;
  params.J_c_0[14] = -1.2144776931994525;
  params.J_c_0[15] = -1.2317108144255875;
  params.J_c_0[16] = 0.9026784957312834;
  params.J_c_0[17] = 1.1397468137245244;
  params.J_c_0[18] = 1.8883934547350631;
  params.J_c_0[19] = 1.4038856681660068;
  params.J_c_0[20] = 0.17437730638329096;
  params.retreat[0] = 0.17958173904612962;
  params.normal_1[0] = -0.04450702153554875;
  params.normal_1[1] = 1.7117453902485025;
  params.normal_1[2] = 1.1504727980139053;
  params.J_c_1[0] = -0.05962309578364744;
  params.J_c_1[1] = -0.1788825540764547;
  params.J_c_1[2] = -1.1280569263625857;
  params.J_c_1[3] = -1.2911464767927057;
  params.J_c_1[4] = -1.7055053231225696;
  params.J_c_1[5] = 1.56957275034837;
  params.J_c_1[6] = 0.5607064675962357;
  params.J_c_1[7] = -1.4266707301147146;
  params.J_c_1[8] = -0.3434923211351708;
  params.J_c_1[9] = -1.8035643024085055;
  params.J_c_1[10] = -1.1625066019105454;
  params.J_c_1[11] = 0.9228324965161532;
  params.J_c_1[12] = 0.6044910817663975;
  params.J_c_1[13] = -0.0840868104920891;
  params.J_c_1[14] = -0.900877978017443;
  params.J_c_1[15] = 0.608892500264739;
  params.J_c_1[16] = 1.8257980452695217;
  params.J_c_1[17] = -0.25791777529922877;
  params.J_c_1[18] = -1.7194699796493191;
  params.J_c_1[19] = -1.7690740487081298;
  params.J_c_1[20] = -1.6685159248097703;
  params.normal_2[0] = 1.8388287490128845;
  params.normal_2[1] = 0.16304334474597537;
  params.normal_2[2] = 1.3498497306788897;
  params.J_c_2[0] = -1.3198658230514613;
  params.J_c_2[1] = -0.9586197090843394;
  params.J_c_2[2] = 0.7679100474913709;
  params.J_c_2[3] = 1.5822813125679343;
  params.J_c_2[4] = -0.6372460621593619;
  params.J_c_2[5] = -1.741307208038867;
  params.J_c_2[6] = 1.456478677642575;
  params.J_c_2[7] = -0.8365102166820959;
  params.J_c_2[8] = 0.9643296255982503;
  params.J_c_2[9] = -1.367865381194024;
  params.J_c_2[10] = 0.7798537405635035;
  params.J_c_2[11] = 1.3656784761245926;
  params.J_c_2[12] = 0.9086083149868371;
  params.J_c_2[13] = -0.5635699005460344;
  params.J_c_2[14] = 0.9067590059607915;
  params.J_c_2[15] = -1.4421315032701587;
  params.J_c_2[16] = -0.7447235390671119;
  params.J_c_2[17] = -0.32166897326822186;
  params.J_c_2[18] = 1.5088481557772684;
  params.J_c_2[19] = -1.385039165715428;
  params.J_c_2[20] = 1.5204991609972622;
  params.normal_3[0] = 1.1958572768832156;
  params.normal_3[1] = 1.8864971883119228;
  params.normal_3[2] = -0.5291880667861584;
  params.J_c_3[0] = -1.1802409243688836;
  params.J_c_3[1] = -1.037718718661604;
  params.J_c_3[2] = 1.3114512056856835;
  params.J_c_3[3] = 1.8609125943756615;
  params.J_c_3[4] = 0.7952399935216938;
  params.J_c_3[5] = -0.07001183290468038;
  params.J_c_3[6] = -0.8518009412754686;
  params.J_c_3[7] = 1.3347515373726386;
  params.J_c_3[8] = 1.4887180335977037;
  params.J_c_3[9] = -1.6314736327976336;
  params.J_c_3[10] = -1.1362021159208933;
  params.J_c_3[11] = 1.327044361831466;
  params.J_c_3[12] = 1.3932155883179842;
  params.J_c_3[13] = -0.7413880049440107;
  params.J_c_3[14] = -0.8828216126125747;
  params.J_c_3[15] = -0.27673991192616;
  params.J_c_3[16] = 0.15778600105866714;
  params.J_c_3[17] = -1.6177327399735457;
  params.J_c_3[18] = 1.3476485548544606;
  params.J_c_3[19] = 0.13893948140528378;
  params.J_c_3[20] = 1.0998712601636944;
  params.normal_4[0] = -1.0766549376946926;
  params.normal_4[1] = 1.8611734044254629;
  params.normal_4[2] = 1.0041092292735172;
  params.J_c_4[0] = -0.6276245424321543;
  params.J_c_4[1] = 1.794110587839819;
  params.J_c_4[2] = 0.8020471158650913;
  params.J_c_4[3] = 1.362244341944948;
  params.J_c_4[4] = -1.8180107765765245;
  params.J_c_4[5] = -1.7774338357932473;
  params.J_c_4[6] = 0.9709490941985153;
  params.J_c_4[7] = -0.7812542682064318;
  params.J_c_4[8] = 0.0671374633729811;
  params.J_c_4[9] = -1.374950305314906;
  params.J_c_4[10] = 1.9118096386279388;
  params.J_c_4[11] = 0.011004190697677885;
  params.J_c_4[12] = 1.3160043138989015;
  params.J_c_4[13] = -1.7038488148800144;
  params.J_c_4[14] = -0.08433819112864738;
  params.J_c_4[15] = -1.7508820783768964;
  params.J_c_4[16] = 1.536965724350949;
  params.J_c_4[17] = -0.21675928514816478;
  params.J_c_4[18] = -1.725800326952653;
  params.J_c_4[19] = -1.6940148707361717;
  params.J_c_4[20] = 0.15517063201268;
  params.normal_5[0] = -1.697734381979077;
  params.normal_5[1] = -1.264910727950229;
  params.normal_5[2] = -0.2545716633339441;
  params.J_c_5[0] = -0.008868675926170244;
  params.J_c_5[1] = 0.3332476609670296;
  params.J_c_5[2] = 0.48205072561962936;
  params.J_c_5[3] = -0.5087540014293261;
  params.J_c_5[4] = 0.4749463319223195;
  params.J_c_5[5] = -1.371021366459455;
  params.J_c_5[6] = -0.8979660982652256;
  params.J_c_5[7] = 1.194873082385242;
  params.J_c_5[8] = -1.3876427970939353;
  params.J_c_5[9] = -1.106708108457053;
  params.J_c_5[10] = -1.0280872812241797;
  params.J_c_5[11] = -0.08197078070773234;
  params.J_c_5[12] = -1.9970179118324083;
  params.J_c_5[13] = -1.878754557910134;
  params.J_c_5[14] = -0.15380739340877803;
  params.J_c_5[15] = -1.349917260533923;
  params.J_c_5[16] = 0.7180072150931407;
  params.J_c_5[17] = 1.1808183487065538;
  params.J_c_5[18] = 0.31265343495084075;
  params.J_c_5[19] = 0.7790599086928229;
  params.J_c_5[20] = -0.4361679370644853;
  params.normal_6[0] = -1.8148151880282066;
  params.normal_6[1] = -0.24231386948140266;
  params.normal_6[2] = -0.5120787511622411;
  params.J_c_6[0] = 0.3880129688013203;
  params.J_c_6[1] = -1.4631273212038676;
  params.J_c_6[2] = -1.0891484131126563;
  params.J_c_6[3] = 1.2591296661091191;
  params.J_c_6[4] = -0.9426978934391474;
  params.J_c_6[5] = -0.358719180371347;
  params.J_c_6[6] = 1.7438887059831263;
  params.J_c_6[7] = -0.8977901479165817;
  params.J_c_6[8] = -1.4188401645857445;
  params.J_c_6[9] = 0.8080805173258092;
  params.J_c_6[10] = 0.2682662017650985;
  params.J_c_6[11] = 0.44637534218638786;
  params.J_c_6[12] = -1.8318765960257055;
  params.J_c_6[13] = -0.3309324209710929;
  params.J_c_6[14] = -1.9829342633313622;
  params.J_c_6[15] = -1.013858124556442;
  params.J_c_6[16] = 0.8242247343360254;
  params.J_c_6[17] = -1.753837136317201;
  params.J_c_6[18] = -0.8212260055868805;
  params.J_c_6[19] = 1.9524510112487126;
  params.J_c_6[20] = 1.884888920907902;
  params.normal_7[0] = -0.0726144452811801;
  params.normal_7[1] = 0.9427735461129836;
  params.normal_7[2] = 0.5306230967445558;
  params.J_c_7[0] = -0.1372277142250531;
  params.J_c_7[1] = 1.4282657305652786;
  params.J_c_7[2] = -1.309926991335284;
  params.J_c_7[3] = 1.3137276889764422;
  params.J_c_7[4] = -1.8317219061667278;
  params.J_c_7[5] = 1.4678147672511939;
  params.J_c_7[6] = 0.703986349872991;
  params.J_c_7[7] = -0.2163435603565258;
  params.J_c_7[8] = 0.6862809905371079;
  params.J_c_7[9] = -0.15852598444303245;
  params.J_c_7[10] = 1.1200128895143409;
  params.J_c_7[11] = -1.5462236645435308;
  params.J_c_7[12] = 0.0326297153944215;
  params.J_c_7[13] = 1.4859581597754916;
  params.J_c_7[14] = 1.71011710324809;
  params.J_c_7[15] = -1.1186546738067493;
  params.J_c_7[16] = -0.9922787897815244;
  params.J_c_7[17] = 1.6160498864359547;
  params.J_c_7[18] = -0.6179306451394861;
  params.J_c_7[19] = -1.7725097038051376;
  params.J_c_7[20] = 0.8595466884481313;
  params.normal_8[0] = -0.3423245633865686;
  params.normal_8[1] = 0.9412967499805762;
  params.normal_8[2] = -0.09163346622652258;
  params.J_c_8[0] = 0.002262217745727657;
  params.J_c_8[1] = -0.3297523583656421;
  params.J_c_8[2] = -0.8380604158593941;
  params.J_c_8[3] = 1.6028434695494038;
  params.J_c_8[4] = 0.675150311940429;
  params.J_c_8[5] = 1.1553293733718686;
  params.J_c_8[6] = 1.5829581243724693;
  params.J_c_8[7] = -0.9992442304425597;
  params.J_c_8[8] = 1.6792824558896897;
  params.J_c_8[9] = 1.4504203490342324;
  params.J_c_8[10] = 0.02434104849994556;
  params.J_c_8[11] = 0.27160869657612263;
  params.J_c_8[12] = -1.5402710478528858;
  params.J_c_8[13] = 1.0484633622310744;
  params.J_c_8[14] = -1.3070999712627054;
  params.J_c_8[15] = 0.13534416402363814;
  params.J_c_8[16] = -1.4942507790851232;
  params.J_c_8[17] = -1.708331625671371;
  params.J_c_8[18] = 0.436109775042258;
  params.J_c_8[19] = -0.03518748153727991;
  params.J_c_8[20] = 0.6992397389570906;
  params.normal_9[0] = 1.1634167322171374;
  params.normal_9[1] = 1.9307499705822648;
  params.normal_9[2] = -1.6636772756932747;
  params.J_c_9[0] = 0.5248484497343218;
  params.J_c_9[1] = 0.30789958152579144;
  params.J_c_9[2] = 0.602568707166812;
  params.J_c_9[3] = 0.17271781925751872;
  params.J_c_9[4] = 0.2294695501208066;
  params.J_c_9[5] = 1.4742185345619543;
  params.J_c_9[6] = -0.1919535345136989;
  params.J_c_9[7] = 0.13990231452144553;
  params.J_c_9[8] = 0.7638548150610602;
  params.J_c_9[9] = -1.6420200344195646;
  params.J_c_9[10] = -0.27229872445076087;
  params.J_c_9[11] = -1.5914631171820468;
  params.J_c_9[12] = -1.4487604283558668;
  params.J_c_9[13] = -1.991497766136364;
  params.J_c_9[14] = -1.1611742553535152;
  params.J_c_9[15] = -1.133450950247063;
  params.J_c_9[16] = 0.06497792493777155;
  params.J_c_9[17] = 0.28083295396097263;
  params.J_c_9[18] = 1.2958447220129887;
  params.J_c_9[19] = -0.05315524470737154;
  params.J_c_9[20] = 1.5658183956871667;
  params.normal_10[0] = -0.41975684089933685;
  params.normal_10[1] = 0.97844578833777;
  params.normal_10[2] = 0.2110290496695293;
  params.J_c_10[0] = 0.4953003430893044;
  params.J_c_10[1] = -0.9184320124667495;
  params.J_c_10[2] = 1.750380031759156;
  params.J_c_10[3] = 1.0786188614315915;
  params.J_c_10[4] = -1.4176198837203735;
  params.J_c_10[5] = 0.149737479778294;
  params.J_c_10[6] = 1.9831452222223418;
  params.J_c_10[7] = -1.8037746699794734;
  params.J_c_10[8] = -0.7887206483295461;
  params.J_c_10[9] = 0.9632534854086652;
  params.J_c_10[10] = -1.8425542093895406;
  params.J_c_10[11] = 0.986684363969033;
  params.J_c_10[12] = 0.2936851199350441;
  params.J_c_10[13] = 0.9268227022482662;
  params.J_c_10[14] = 0.20333038350653299;
  params.J_c_10[15] = 1.7576139132046351;
  params.J_c_10[16] = -0.614393188398918;
  params.J_c_10[17] = 0.297877839744912;
  params.J_c_10[18] = -1.796880083990895;
  params.J_c_10[19] = 0.21373133661742738;
  params.J_c_10[20] = -0.32242822540825156;
  params.normal_11[0] = 1.9326471511608059;
  params.normal_11[1] = 1.7824292753481785;
  params.normal_11[2] = -1.4468823405675986;
  params.J_c_11[0] = -1.8335374338761512;
  params.J_c_11[1] = -1.5172997317243713;
  params.J_c_11[2] = -1.229012129120719;
  params.J_c_11[3] = 0.9046719772422094;
  params.J_c_11[4] = 0.17591181415489432;
  params.J_c_11[5] = 0.13970133814112584;
  params.J_c_11[6] = -0.14185208214985234;
  params.J_c_11[7] = -1.9732231264739348;
  params.J_c_11[8] = -0.4301123458221334;
  params.J_c_11[9] = 1.9957537650387742;
  params.J_c_11[10] = 1.2811648216477893;
  params.J_c_11[11] = 0.2914428437588219;
  params.J_c_11[12] = -1.214148157218884;
  params.J_c_11[13] = 1.6818776980374155;
  params.J_c_11[14] = -0.30341101038214635;
  params.J_c_11[15] = 0.47730909231793106;
  params.J_c_11[16] = -1.187569373035299;
  params.J_c_11[17] = -0.6877370247915531;
  params.J_c_11[18] = -0.6201861482616171;
  params.J_c_11[19] = -0.4209925183921568;
  params.J_c_11[20] = -1.9110724537712471;
  params.normal_12[0] = 0.6413882087807936;
  params.normal_12[1] = -1.3200399280087032;
  params.normal_12[2] = 0.41320105301312626;
  params.J_c_12[0] = 0.4783213861392275;
  params.J_c_12[1] = 0.7916189857293743;
  params.J_c_12[2] = -0.8322752558146558;
  params.J_c_12[3] = -0.8318720537426154;
  params.J_c_12[4] = 1.0221179076113445;
  params.J_c_12[5] = -0.4471032189262627;
  params.J_c_12[6] = -1.3901469561676985;
  params.J_c_12[7] = 1.6210596051208572;
  params.J_c_12[8] = -1.9476687601912737;
  params.J_c_12[9] = 1.5459376306231292;
  params.J_c_12[10] = -0.830972896191656;
  params.J_c_12[11] = -0.47269983955176276;
  params.J_c_12[12] = 1.913620609584223;
  params.J_c_12[13] = -0.25329703423935124;
  params.J_c_12[14] = 0.8635279149674653;
  params.J_c_12[15] = -0.35046893227111564;
  params.J_c_12[16] = 1.6541432486772365;
  params.J_c_12[17] = 0.8779619968413503;
  params.J_c_12[18] = -0.07723284625844862;
  params.J_c_12[19] = -1.6631134040635196;
  params.J_c_12[20] = -0.54546452868516;
  params.normal_13[0] = -0.03757319061095998;
  params.normal_13[1] = -0.864543266194465;
  params.normal_13[2] = 0.13856203767859343;
  params.J_c_13[0] = -1.1613957272733684;
  params.J_c_13[1] = -0.022681697832835024;
  params.J_c_13[2] = 0.11202078062843634;
  params.J_c_13[3] = 0.6934385624164641;
  params.J_c_13[4] = 0.9814633803279791;
  params.J_c_13[5] = 0.9198949681022897;
  params.J_c_13[6] = -0.3035363988458051;
  params.J_c_13[7] = -0.1761906755724203;
  params.J_c_13[8] = 1.4940284058791686;
  params.J_c_13[9] = -0.5488483097174393;
  params.J_c_13[10] = 0.9521313238305416;
  params.J_c_13[11] = 1.9762689267600413;
  params.J_c_13[12] = 1.6992335341478482;
  params.J_c_13[13] = 0.1969474711697119;
  params.J_c_13[14] = -0.7795544525014559;
  params.J_c_13[15] = 0.4892505434034007;
  params.J_c_13[16] = 0.7372066729248594;
  params.J_c_13[17] = 0.10784901966517557;
  params.J_c_13[18] = -0.6340934767066218;
  params.J_c_13[19] = -0.17829371464242083;
  params.J_c_13[20] = -1.6728370279392784;
  params.normal_14[0] = -0.8348711800042916;
  params.normal_14[1] = -1.4204129800590897;
  params.normal_14[2] = 0.6659229232859376;
  params.J_c_14[0] = 1.8369365661533168;
  params.J_c_14[1] = -1.371061267737546;
  params.J_c_14[2] = -1.8868237125934915;
  params.J_c_14[3] = 0.9654286768651104;
  params.J_c_14[4] = -0.5833420409292005;
  params.J_c_14[5] = 0.02386510653728502;
  params.J_c_14[6] = -1.7558076992858345;
  params.J_c_14[7] = -1.2889402130475411;
  params.J_c_14[8] = 0.7820251677632606;
  params.J_c_14[9] = 0.4208424784688227;
  params.J_c_14[10] = 1.4136448896755982;
  params.J_c_14[11] = 1.8516928541530757;
  params.J_c_14[12] = -0.5615396035790421;
  params.J_c_14[13] = 0.4809940266433177;
  params.J_c_14[14] = -0.20929035114697303;
  params.J_c_14[15] = 0.022387850798402553;
  params.J_c_14[16] = -0.43399296564115764;
  params.J_c_14[17] = 1.9095769077945013;
  params.J_c_14[18] = 0.4945512698336847;
  params.J_c_14[19] = -1.4324582900293557;
  params.J_c_14[20] = 0.790913765746676;
  params.normal_15[0] = 1.8630250293383734;
  params.normal_15[1] = 1.5793975466121069;
  params.normal_15[2] = 0.2320163334712646;
  params.J_c_15[0] = -1.9411408650055968;
  params.J_c_15[1] = 1.2221853270725478;
  params.J_c_15[2] = 1.7274453600045607;
  params.J_c_15[3] = 0.9357159281665783;
  params.J_c_15[4] = -0.2841874908331623;
  params.J_c_15[5] = -0.4766355664552626;
  params.J_c_15[6] = 0.9784190546201912;
  params.J_c_15[7] = -1.5685956114005477;
  params.J_c_15[8] = 1.1387833891036;
  params.J_c_15[9] = -0.004779126480003892;
  params.J_c_15[10] = -1.7195239474925414;
  params.J_c_15[11] = 1.2921808565147272;
  params.J_c_15[12] = -0.43317009071966606;
  params.J_c_15[13] = -1.572940257279357;
  params.J_c_15[14] = -1.3048062231674988;
  params.J_c_15[15] = 1.4377304631579175;
  params.J_c_15[16] = -1.3090328020145874;
  params.J_c_15[17] = 1.1370018620707785;
  params.J_c_15[18] = 1.2164644012668289;
  params.J_c_15[19] = -1.6539274174499985;
  params.J_c_15[20] = -0.25845368809725544;
  params.q_min[0] = 1.1486358936399745;
  params.q_min[1] = -0.03975647517318137;
  params.q_min[2] = 1.4640632749164326;
  params.q_min[3] = -0.48111499989733186;
  params.q_min[4] = 0.5132576752843594;
  params.q_min[5] = -1.1459189400462249;
  params.q_min[6] = 1.3690255364554855;
  params.has_limits[0] = 1.6787145728001627;
  params.has_limits[1] = 1.1316686691151863;
  params.has_limits[2] = 0.6461768932356984;
  params.has_limits[3] = 0.6951363818273177;
  params.has_limits[4] = 1.1893654840755423;
  params.has_limits[5] = 0.25681815327072943;
  params.has_limits[6] = 1.0209456791690243;
  params.q_max[0] = -0.8182949160834703;
  params.q_max[1] = -0.6336865828985854;
  params.q_max[2] = -0.7126437991119396;
  params.q_max[3] = 1.3381487344587226;
  params.q_max[4] = -1.2979975504895949;
  params.q_max[5] = -1.0542097271412714;
  params.q_max[6] = -1.3421003125955435;
}
