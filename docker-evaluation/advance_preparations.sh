mkdir -p output
cp -r ../aichallenge/AWSIM ./
tar zcvf aichallenge_submit.tar.gz -C ../aichallenge/autoware/src aichallenge_submit
