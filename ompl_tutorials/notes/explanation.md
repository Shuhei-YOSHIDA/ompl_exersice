OMPL に関するメモ
====

* StateSpaceクラスの各空間に対する実装を指すSE3StateSpaceやRealVectorStateSpace クラスにはそのクラス内クラスとしてStateTypeというクラスがあり，
setXYZや配列添字のオペレータといったsetter/getterが準備されている

* simplifySolution を実行しないとかなり経路は複雑になる

* Optimal Planningのためには
  1. 目的関数を `ompl::base::ProblemDefinition` に指定する
  1. 最適化を行うPlannerを選択すること(A-star とかPRM-starとか)
  1. でないと `simpleSetup::getProblemDefinition()` とかすると `invalid use of incomplete ...`  のようにテンプレートが指定されていないというエラーがでてコンパイルできない


* `ompl::base::StateValidityChecker` はあるサンプリング点が有効かをしらべる

* `ompl::base::MotionValidator` はnode間をつなぐpathが有効かどうかを調べる．
しかし，大抵DiscreteMotionValidatorがデフォルトで設定されていて，path間に複数の点をとって，
その点のValidityを調べることでpathチェックが行われているので，実装の必要性は低い

* planner側に振る舞いを関数を渡して設定することがある．
例えばPRMにconnectionFilterやconnectionStrategyを設定できる．
これはPlannerがサンプリングを行うときのレベルでの振る舞い．

* PRMでRoadmap上のnodeとパスのみで経路を構成したいとき(simplifySolutionを使いたくない時),
SE3だと，position分の経路がかなりごちゃごちゃになるが，OptimizationObjectiveとして最小パスを設定してやればだいぶマシになる

* simplifySolutionを行うと，Roadmap上のnodeから外れた経路を生成するように見える．どういう実装？
