#!/usr/bin/env groovy
@Library('tailor-meta@0.1.5')_
tailorTestPipeline(
  // Name of job that generated this test definition.
  rosdistro_job: '/ci/rosdistro/release%2F19.1',
  // Distribution name
  rosdistro_name: 'ros1',
  // Release track to test branch against.
  release_track: '19.1',
  // Release label to pull test images from.
  release_label: '19.1-rc',
  // OS distributions to test.
  distributions: ['xenial', 'bionic'],
  // Bundle flavour to test against.
  flavour: 'dev',
  // Branch of tailor_meta to build against
  tailor_meta_branch: '0.1.5',
  // Master or release branch associated with this track
  source_branch: 'release/19.1',
  // Docker registry where test image is stored
  docker_registry: 'https://084758475884.dkr.ecr.us-east-1.amazonaws.com/locus-tailor'
)