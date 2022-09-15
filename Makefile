build:
	docker build -t gmto.im/windloading .
run:
	docker run --rm gmto.im/windloading
push:
	aws ecr get-login-password --region us-west-2 | docker login --username AWS --password-stdin 378722409401.dkr.ecr.us-west-2.amazonaws.com
	docker tag gmto.im/windloading:latest 378722409401.dkr.ecr.us-west-2.amazonaws.com/gmto.im/windloading:latest
	docker push 378722409401.dkr.ecr.us-west-2.amazonaws.com/gmto.im/windloading:latest
stack:
	aws s3 cp windloading.yaml s3://gmto.modeling/stacks/
	aws cloudformation create-stack --stack-name windloading --template-url https://s3-us-west-2.amazonaws.com/gmto.modeling/stacks/windloading.yaml --region us-west-2
